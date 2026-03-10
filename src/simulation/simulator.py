"""
simulator.py  –  Main simulation engine (FixedWingSimulator).

Orchestrates all modules:
  models      → aircraft parameters
  dynamics    → nonlinear 6-DOF ODE
  environment → wind + atmosphere
  control     → 5-layer ArduPilot control chain
  planning    → trajectory / waypoint manager
  simulation  → integrator, state manager, history

Supports two simulation modes:
  run()             – closed-loop real-time-step loop
  run_linear_analysis() – 4-DOF linear open-loop analysis (backward-compatible)
"""

from __future__ import annotations

import os
import sys
import numpy as np
from typing import Optional, List, Dict, Any

# ------------------------------------------------------------------
# Resolve src/ on sys.path so sibling packages are importable
# regardless of how main.py is launched.
# ------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC  = os.path.dirname(_HERE)
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from models.aircraft_factory      import AircraftFactory, AircraftConfig
from models.aircraft_database     import AIRCRAFT_NAMES
from dynamics.linear_model        import LinearModel, LinearAnalysisResult
from dynamics.nonlinear_model     import NonlinearModel, Controls, TrimResult
from dynamics.coordinate_transform import wind_to_body_frame
from environment.wind_model       import Wind
from environment.atmosphere_model import compute_density
from control.ardupilot_compat     import ArdupilotParams
from control.flight_mode_manager  import (
    FlightMode, FlightModeManager, AircraftState, ControlTarget,
)
from control.navigation_controller import NavigationController, PathSegment
from control.attitude_controller  import AttitudeController
from control.rate_controller      import RateController
from control.servo_mixer          import ServoMixer, ServoOutput
from planning.waypoint_manager    import WaypointManager
from simulation.integrator        import Dopri5Integrator
from simulation.state_manager     import AircraftSimState, StateHistory
from utils.config_loader          import ConfigLoader
from utils.math_utils             import rotation_matrix_321


# ---------------------------------------------------------------------------
# Simulation result container
# ---------------------------------------------------------------------------

class SimulationResult:
    """
    Container for a complete simulation run.

    Wraps StateHistory and provides convenience methods for summary and plotting.
    """

    def __init__(
        self,
        history:    StateHistory,
        trim:       TrimResult,
        uav_name:   str,
        closed_loop: bool,
    ):
        self.history     = history
        self.trim        = trim
        self.uav_name    = uav_name
        self.closed_loop = closed_loop

    def summary(self) -> str:
        h = self.history.to_dict()
        t = h["t"]
        lines = [
            f"=== {self.uav_name} Simulation Result ===",
            f"  Trim speed : {self.trim.U0:.2f} m/s ({self.trim.U0*1.944:.2f} kn)",
            f"  Duration   : {t[-1]:.1f} s  | Steps: {len(t)}",
            f"  Mode       : {'Closed-loop' if self.closed_loop else 'Open-loop'}",
            f"  Final alt  : {h['altitude'][-1]:.1f} m",
            f"  Final speed: {h['airspeed'][-1]:.1f} m/s",
            f"  Track (N,E): ({h['x_north'][-1]:.0f}, {h['x_east'][-1]:.0f}) m",
        ]
        return "\n".join(lines)

    def visualize(self, show: bool = True) -> None:
        """Quick 2D + 3D visualisation using the plotter and animator."""
        try:
            import sys, os
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
            from visualization.plotter  import FixedWingPlotter
            from visualization.animator import FixedWingAnimator
        except ImportError as e:
            print(f"[SimulationResult] Visualisation not available: {e}")
            return

        h = self.history.to_dict()
        plotter  = FixedWingPlotter()
        animator = FixedWingAnimator()

        plotter.plot_6dof_matplotlib(h, self.uav_name, show=show)
        animator.animate(h, self.uav_name, show=show)


# ---------------------------------------------------------------------------
# Main simulator class
# ---------------------------------------------------------------------------

class FixedWingSimulator:
    """
    Fixed-wing UAV simulator with ArduPilot/PX4-compatible control.

    Parameters
    ----------
    aircraft_name    : aircraft key from the database
    config_dir       : path to config/ directory
    dt               : simulation time step (s)
    duration         : total simulation duration (s)
    initial_mode     : starting FlightMode (string or FlightMode enum)
    wind_type        : 'NONE' | 'FIXED' | 'SINE' | 'RANDOMSINE'
    traj_type        : 'minimum_snap' | 'minimum_jerk'
    controller_type  : 'ardupilot' | 'px4' - 控制器类型
    """

    def __init__(
        self,
        aircraft_name: str  = "TB2",
        config_dir:    str  = None,
        dt:            float = 0.01,
        duration:      float = 30.0,
        initial_mode:  str  = "AUTO",
        wind_type:     str  = "NONE",
        traj_type:     str  = "minimum_snap",
        controller_type: str = "ardupilot",  # 新增：控制器类型 ("ardupilot" 或 "px4")
    ):
        if aircraft_name not in AIRCRAFT_NAMES:
            raise ValueError(f"Unknown aircraft '{aircraft_name}'. Available: {AIRCRAFT_NAMES}")

        self.dt       = dt
        self.duration = duration
        self.wind_type = wind_type
        self.traj_type = traj_type

        # --- Config ---------------------------------------------------------
        if config_dir is None:
            config_dir = os.path.join(os.path.dirname(__file__), "..", "..", "config")
        config_dir = os.path.abspath(config_dir)
        self._cfg = ConfigLoader(config_dir)
        sim_cfg   = self._cfg.load_simulation()
        
        # 从配置文件读取控制器类型（如果未在参数中指定）
        if controller_type == "ardupilot":  # 默认值
            self.controller_type = sim_cfg.get("controller_type", "ardupilot").lower()
        else:
            self.controller_type = controller_type.lower()
        
        if self.controller_type not in ["ardupilot", "px4"]:
            raise ValueError(f"Unknown controller type '{self.controller_type}'. Must be 'ardupilot' or 'px4'.")

        # --- Aircraft -------------------------------------------------------
        self.aircraft_cfg: AircraftConfig = AircraftFactory.create(aircraft_name)
        self.params = self.aircraft_cfg.aero_params

        # --- Environment ----------------------------------------------------
        env_wind_type  = wind_type or sim_cfg.get("wind_type", "NONE")
        wind_speed     = sim_cfg.get("wind_speed", 5.0)
        wind_dir_deg   = sim_cfg.get("wind_direction_deg", 270.0)
        self.wind = Wind(env_wind_type, speed=wind_speed, direction_deg=wind_dir_deg)

        # --- Control parameters ---------------------------------
        # 根据控制器类型加载不同的配置文件
        if self.controller_type == "px4":
            ctrl_path = os.path.join(config_dir, "px4_control_params.yaml")
            # 如果 PX4 配置文件不存在，回退到 ArduPilot 配置
            if not os.path.isfile(ctrl_path):
                ctrl_path = os.path.join(config_dir, "control_params.yaml")
        else:  # ardupilot
            ctrl_path = os.path.join(config_dir, "control_params.yaml")
        
        if os.path.isfile(ctrl_path):
            self.ap_params = ArdupilotParams.from_yaml(ctrl_path)
        else:
            self.ap_params = ArdupilotParams()
        self.ap_params.validate()

        # --- Control layers -------------------------------------------------
        mode_enum = FlightMode(initial_mode.upper())
        self.mode_mgr = FlightModeManager(
            initial_mode=mode_enum,
            cruise_speed=self.ap_params.AIRSPEED_CRUISE,
            cruise_alt=self.ap_params.ALT_HOLD_RTL,
        )
        # ---- Load TECS params from YAML (with defaults) ------------------
        _raw_yaml: dict = {}
        if os.path.isfile(ctrl_path):
            import yaml as _yaml
            with open(ctrl_path) as _f:
                _raw_yaml = _yaml.safe_load(_f) or {}

        def _tp(key, default):
            return float(_raw_yaml.get(key, default))

        self.nav_ctrl = NavigationController(
            l1_period           = self.ap_params.NAVL1_PERIOD,
            l1_damping          = self.ap_params.NAVL1_DAMPING,
            max_roll            = np.radians(self.ap_params.LIM_ROLL_DEG),
            cruise_speed        = self.ap_params.AIRSPEED_CRUISE,
            cruise_alt          = self.ap_params.ALT_HOLD_RTL,
            tecs_max_climb_rate = _tp("TECS_CLMB_MAX",    5.0),
            tecs_min_sink_rate  = _tp("TECS_SINK_MIN",    2.0),
            tecs_max_sink_rate  = _tp("TECS_SINK_MAX",    5.0),
            tecs_time_const     = _tp("TECS_TIME_CONST",  5.0),
            tecs_thr_damp       = _tp("TECS_THR_DAMP",    0.5),
            tecs_ptch_damp      = _tp("TECS_PTCH_DAMP",   0.3),
            tecs_integ_gain     = _tp("TECS_INTEG_GAIN",  0.3),
            tecs_spd_weight     = _tp("TECS_SPDWEIGHT",   1.0),
            tecs_roll_comp      = _tp("TECS_RLL2THR",    10.0),
            tecs_pitch_min      = np.radians(_tp("TECS_PITCH_MIN", -15.0)),
            tecs_pitch_max      = np.radians(_tp("TECS_PITCH_MAX",  15.0)),
            tecs_thr_cruise     = _tp("TECS_THR_CRUISE",  0.36),
            tecs_thr_min        = self.ap_params.THR_MIN,
            tecs_thr_max        = self.ap_params.THR_MAX,
            airspeed_min        = _tp("AIRSPEED_MIN",     28.0),
            airspeed_max        = _tp("AIRSPEED_MAX",     60.0),
            tecs_hdem_tconst    = _tp("TECS_HDEM_TCONST", 1.5),
        )
        # --- 根据控制器类型创建姿态和速率控制器 ---
        if self.controller_type == "px4":
            from control.px4_attitude_controller import PX4AttitudeController
            from control.px4_rate_controller import PX4RateController
            
            self.att_ctrl = PX4AttitudeController(dt=dt)
            self.rate_ctrl = PX4RateController(dt=dt)
            
            # 从配置文件读取 PX4 参数
            # 姿态外环：时间常数
            self.att_ctrl.set_time_constants(
                roll_tc=_tp("FW_R_TC", 0.5),
                pitch_tc=_tp("FW_P_TC", 0.5),
                yaw_tc=_tp("FW_Y_TC", 0.5)
            )
            
            # 速率内环：P, I, FF
            self.rate_ctrl.set_all_gains(
                roll_gains={'kp': _tp("FW_RR_P", 0.06), 
                           'ki': _tp("FW_RR_I", 0.01),
                           'kff': _tp("FW_RR_FF", 0.4)},
                pitch_gains={'kp': _tp("FW_PR_P", 0.04),
                            'ki': _tp("FW_PR_I", 0.01),
                            'kff': _tp("FW_PR_FF", 0.4)},
                yaw_gains={'kp': _tp("FW_YR_P", 0.05),
                          'ki': _tp("FW_YR_I", 0.01),
                          'kff': _tp("FW_YR_FF", 0.3)}
            )
            
            # 设置角速率限制
            self.att_ctrl.max_roll_rate = np.radians(_tp("FW_R_RMAX", 120.0))
            self.att_ctrl.max_pitch_rate = np.radians(_tp("FW_P_RMAX", 60.0))
            self.att_ctrl.max_yaw_rate = np.radians(_tp("FW_Y_RMAX", 45.0))
        else:  # ardupilot
            self.att_ctrl  = AttitudeController(self.ap_params, dt=dt)
            self.rate_ctrl = RateController(self.ap_params, dt=dt)
        
        self.servo = ServoMixer(self.ap_params, dt=dt)

        # --- Trajectory / planning ------------------------------------------
        self.wp_mgr = WaypointManager(
            average_speed=self.ap_params.AIRSPEED_CRUISE,
            traj_type=traj_type,
            yaw_mode="yaw_follow",
        )
        # NOTE: trajectory.yaml is NOT auto-loaded here.
        # Users call sim.wp_mgr.add_waypoint() or sim.wp_mgr.load_from_yaml()
        # before calling sim.run().  This avoids the default cruise-circle
        # waypoints (all at 100 m) contaminating user-defined missions.
        # If you want the built-in trajectory, call:
        #   sim.wp_mgr.load_from_yaml(<config_dir>/trajectory.yaml)

        # --- Dynamics -------------------------------------------------------
        self.dyn = NonlinearModel(self.params)
        self._trim: Optional[TrimResult] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(
        self,
        closed_loop:    bool = True,
        use_trajectory: bool = True,
        wp_switch_dist: float = 60.0,
        loop_circuit:   bool = False,
    ) -> SimulationResult:
        """
        Run the complete simulation.

        Parameters
        ----------
        closed_loop    : if True, use the 5-layer ArduPilot control system;
                         if False, run open-loop (trim-hold) dynamics only.
        use_trajectory : if True (default), build a minimum-snap/jerk polynomial
                         trajectory and track it.
                         if False, use simple waypoint-sequencing mode: the L1
                         controller flies directly toward each waypoint in order,
                         switching to the next when within *wp_switch_dist* metres.
                         This is similar to ArduPilot's AUTO mode mission waypoints.
        wp_switch_dist : (use_trajectory=False only) horizontal distance threshold
                         (metres) at which the navigator switches to the next
                         waypoint.  Default 60 m.
        loop_circuit   : (use_trajectory=False only) if True, after reaching the
                         last waypoint the sequence wraps back to waypoint 0 and
                         repeats indefinitely until the simulation duration expires.

        Returns
        -------
        SimulationResult
        """
        trim  = self.dyn.compute_trim()
        self._trim = trim
        n_steps = int(self.duration / self.dt) + 1
        history = StateHistory(n_steps)

        # --- Auto-compute thr_cruise from trim if YAML value is wrong ----------
        # Force balance in body-frame X at trim (level flight, gamma=0):
        #   T + X_aero + X_gravity = 0
        #   X_aero = q_bar*S*(-CD*cos(a) + CL*sin(a))  [aerodynamics.py convention]
        #   X_gravity = -m*g*sin(theta) = -m*g*sin(alpha)   [theta=alpha at trim]
        #   At trim: CL*q_bar*S ≈ m*g  → X_aero ≈ q_bar*S*CD*(-cos(a)) + m*g*sin(a)
        #   ⟹  T_cruise = q_bar*S*CD*cos(alpha_trim)
        _p     = self.dyn._p
        _m     = _p["mass"]
        _G     = 9.80665
        _T_max = _m * _G * 0.20   # same TWR as in state_dot()
        _rho   = _p.get("rho", 1.225)
        _V     = trim.U0
        _S     = _p["S"]
        _alpha = trim.alpha_trim
        _qbar  = 0.5 * _rho * _V**2
        _CD    = _p["CD_0"] + _p["CD_alpha"] * _alpha
        _thr_cruise_auto = _qbar * _S * _CD * np.cos(_alpha) / max(_T_max, 1.0)
        _thr_cruise_auto = float(np.clip(_thr_cruise_auto, 0.0, 1.0))
        # Update TECS cruise throttle to match this aircraft's trim
        if abs(self.nav_ctrl.tecs.thr_cruise - _thr_cruise_auto) > 0.05:
            print(f"[Simulator] Auto-updating thr_cruise: "
                  f"{self.nav_ctrl.tecs.thr_cruise:.3f} → {_thr_cruise_auto:.3f} "
                  f"(aircraft: {self.aircraft_cfg.name})")
            self.nav_ctrl.tecs.thr_cruise = _thr_cruise_auto

        # Initial state
        u0 = trim.U0 * np.cos(trim.alpha_trim)
        w0 = trim.U0 * np.sin(trim.alpha_trim)
        y0 = np.array([
            u0, 0.0, w0,
            0.0, 0.0, 0.0,
            0.0, trim.alpha_trim, 0.0,
            0.0, 0.0, -self.ap_params.ALT_HOLD_RTL,
        ])

        # Reset all control-layer integrators before starting
        # Pass initial state so TECS starts with correct height/airspeed
        self.att_ctrl.reset()
        self.rate_ctrl.reset()
        self.servo.reset()
        from control.flight_mode_manager import AircraftState as _ACS
        _init_state = _ACS(
            altitude=self.ap_params.ALT_HOLD_RTL,
            airspeed=trim.U0,
            phi=0.0, theta=trim.alpha_trim,
            pos_north=0.0, pos_east=0.0, pos_down=-self.ap_params.ALT_HOLD_RTL,
            u=u0, v=0.0, w=w0, p=0.0, q=0.0, r=0.0, psi=0.0,
        )
        self.nav_ctrl.reset(state=_init_state)   # TECS starts at trim state

        # Build a dynamic ODE function that references the control system
        ctrl_holder = [Controls(v_left=trim.de_trim, v_right=trim.de_trim, throttle=0.5)]

        def f_ode(t, y):
            ctrl = ctrl_holder[0]
            wind_ned  = self.wind.get_wind_ned(t)
            phi, theta, psi = y[6], y[7], y[8]
            R = rotation_matrix_321(phi, theta, psi)
            wind_body = R.T @ wind_ned
            alt = -y[11]
            rho = compute_density(alt)
            return self.dyn.state_dot(t, y, ctrl, wind_body=wind_body, rho=rho)

        integrator = Dopri5Integrator(f_ode, y0, t0=0.0)

        # ------------------------------------------------------------------
        # Waypoint / trajectory setup
        # ------------------------------------------------------------------
        _n_wps = len(self.wp_mgr._waypoints_ned)

        if not use_trajectory:
            # ---- Simple waypoint-sequencing mode (no polynomial trajectory) ----
            # The navigator flies directly toward each waypoint in NED space and
            # switches to the next one when within wp_switch_dist metres (2-D).
            if _n_wps == 0:
                print("[Simulator] Circuit mode: no waypoints defined – flying straight.")
                _circuit_wps = []
            else:
                _circuit_wps = list(self.wp_mgr._waypoints_ned)   # copy
                print(f"[Simulator] Circuit mode: {len(_circuit_wps)} waypoints, "
                      f"switch dist={wp_switch_dist:.0f} m")
                for _i, _wp in enumerate(_circuit_wps):
                    print(f"  WP{_i+1}: N={_wp[0]:.0f} E={_wp[1]:.0f} "
                          f"Alt={float(-_wp[2]):.0f} m")
            # Start immediately tracking the *first leg* (WP1 → WP2).
            # WP0 (index 0) is the home/start waypoint co-located with the
            # aircraft's initial position, so there is nothing to fly to there.
            # Skipping it avoids an instant dist=0 switch at t=0 that would
            # cause the first leg to be skipped entirely.
            if len(_circuit_wps) >= 2:
                _wp_idx  = 1          # first *target* waypoint
                _wp_prev = _circuit_wps[0].copy()  # start of first leg = WP1
            else:
                _wp_idx  = 0
                _wp_prev = y0[9:12].copy()
            # Set last-switch time to 0 so the cooldown applies from t=0.
            _wp_last_switch_t = 0.0
            traj_available = False  # not used in circuit mode
            traj = None
        else:
            # ---- Polynomial trajectory mode (original behaviour) ----
            # Ensure trajectory is built.
            # If the first waypoint altitude differs from the actual initial altitude,
            # patch it so the trajectory starts from where the aircraft actually is.
            # This prevents MinSnap from generating a descent leg to reach a 0-m
            # starting waypoint when the aircraft is initialised at ALT_HOLD_RTL.
            #
            # Single-waypoint case: STABILIZE / FBW_B mode can still use TECS for
            # altitude hold toward a single target waypoint.  We synthesize a trivial
            # straight-ahead segment pointing north so TECS gets the correct hgt_dem.
            if _n_wps >= 2:
                _wp0 = self.wp_mgr._waypoints_ned[0]
                _init_alt_ned = -self.ap_params.ALT_HOLD_RTL   # NED down (negative)
                if abs(_wp0[2] - _init_alt_ned) > 1.0:
                    self.wp_mgr._waypoints_ned[0] = np.array([_wp0[0], _wp0[1], _init_alt_ned])
                    self.wp_mgr._trajectory = None  # invalidate cached trajectory
                traj = self.wp_mgr.trajectory
                traj_available = True
            elif _n_wps == 1:
                # Single waypoint: treat it as an altitude hold target.
                _wp_solo = self.wp_mgr._waypoints_ned[0]
                _init_alt_ned = -self.ap_params.ALT_HOLD_RTL
                _wp_start_ned = np.array([0.0, 0.0, _init_alt_ned])
                _wp_end_ned   = np.array([1000.0, 0.0, _wp_solo[2]])
                self.wp_mgr._waypoints_ned = [_wp_start_ned, _wp_end_ned]
                self.wp_mgr._trajectory = None
                traj = self.wp_mgr.trajectory
                traj_available = True
                print(f"[Simulator] Single waypoint detected – altitude hold mode: "
                      f"target alt={float(-_wp_solo[2]):.1f} m")
            else:
                traj_available = False
                traj = None

        t = 0.0
        while t <= self.duration:
            y = integrator.y
            state = AircraftSimState.from_array(y)

            # Convert to AircraftState for the control system
            ac_state = AircraftState(
                pos_north=state.x_north,
                pos_east=state.x_east,
                pos_down=state.x_down,
                u=state.u, v=state.v, w=state.w,
                phi=state.phi, theta=state.theta, psi=state.psi,
                p=state.p, q=state.q, r=state.r,
                airspeed=state.airspeed,
                altitude=state.altitude,
            )

            # --- Control computation (closed-loop) --------------------------
            servo_out = ServoOutput(throttle=0.5)
            nav_target: Optional[ControlTarget] = None

            if closed_loop and not use_trajectory and len(_circuit_wps) > 0:
                # ---- Circuit / waypoint-sequencing navigation ----------------
                # Advance to next waypoint when close enough (2-D horizontal dist).
                # _wp_prev tracks the *previous* waypoint so L1 can track the
                # full segment (prev→current) and compute cross-track error
                # correctly, enabling anticipatory turning before reaching WP.
                _wp_ned = _circuit_wps[_wp_idx]
                _dx = state.x_north - _wp_ned[0]
                _dy = state.x_east  - _wp_ned[1]
                _dist2d = float(np.sqrt(_dx**2 + _dy**2))
                # Waypoint switch criterion: either within wp_switch_dist OR
                # the aircraft has passed the waypoint (dot product of
                # (wp - prev_wp) and (wp - ac_pos) becomes negative, meaning
                # the aircraft is now "past" the waypoint along the track).
                _seg_vec_ne = np.array([_wp_ned[0] - _wp_prev[0],
                                        _wp_ned[1] - _wp_prev[1]])
                _to_wp_ne   = np.array([_wp_ned[0] - state.x_north,
                                        _wp_ned[1] - state.x_east])
                _past_wp    = float(np.dot(_seg_vec_ne, _to_wp_ne)) < 0.0
                # Switch with a minimum cooldown of 5 s to prevent rapid
                # re-switching when near or just past a waypoint.
                _wp_switch_cooldown = 5.0   # s
                if ((_dist2d < wp_switch_dist or _past_wp)
                        and (t - _wp_last_switch_t) > _wp_switch_cooldown):
                    _wp_prev = _wp_ned           # old target becomes new segment start
                    if _wp_idx < len(_circuit_wps) - 1:
                        _wp_idx += 1
                    elif loop_circuit:
                        _wp_idx = 1              # skip WP1 (=WP5 home) → WP2 on loop
                    _wp_ned  = _circuit_wps[_wp_idx]
                    _wp_last_switch_t = t
                    print(f"  t={t:6.1f}s  WP reached → flying to "
                          f"WP{_wp_idx+1}: N={_wp_ned[0]:.0f} E={_wp_ned[1]:.0f} "
                          f"Alt={float(-_wp_ned[2]):.0f} m")

                # Build a path segment from the *previous* waypoint to the active
                # waypoint.  This lets L1 track the full leg and begin banking
                # early rather than pure-chasing toward a point.
                seg = PathSegment(
                    start=_wp_prev,
                    end=_wp_ned,
                    target_speed=self.ap_params.AIRSPEED_CRUISE,
                )
                nav_target = self.nav_ctrl.update(ac_state, seg, dt=self.dt)
                if nav_target is not None:
                    nav_target.pitch_cmd += trim.alpha_trim

            elif closed_loop and traj_available:
                des = traj.desired_state(t)

                # Clamp desired altitude to the bounds of the active path segment.
                _wp_start, _wp_end, _ = self.wp_mgr.get_active_segment(t)
                _seg_alt_a = float(-_wp_start[2])
                _seg_alt_b = float(-_wp_end[2])
                _seg_alt_lo = min(_seg_alt_a, _seg_alt_b)
                _seg_alt_hi = max(_seg_alt_a, _seg_alt_b)
                _des_pos = des.pos.copy()
                _des_pos[2] = float(np.clip(_des_pos[2], -_seg_alt_hi, -_seg_alt_lo))

                seg = PathSegment(
                    start=state.pos_ned,
                    end=_des_pos,
                    target_speed=self.ap_params.AIRSPEED_CRUISE,
                )
                nav_target = self.nav_ctrl.update(ac_state, seg, dt=self.dt)
                if nav_target is not None:
                    nav_target.pitch_cmd += trim.alpha_trim

            ctrl_target = self.mode_mgr.update(ac_state, nav_target, dt=self.dt)

            if not ctrl_target.is_direct and closed_loop:
                att_out  = self.att_ctrl.update(
                    state.phi, state.theta, state.psi,
                    ctrl_target.roll_cmd,
                    ctrl_target.pitch_cmd,
                    ctrl_target.yaw_cmd,
                    dt=self.dt,
                )
                rate_out = self.rate_ctrl.update(
                    state.p, state.q, state.r,
                    att_out.roll_rate_cmd,
                    att_out.pitch_rate_cmd,
                    att_out.yaw_rate_cmd,
                    dt=self.dt,
                )
                servo_out = self.servo.update(
                    rate_out.elevator, rate_out.aileron, rate_out.rudder,
                    ctrl_target.throttle_cmd,
                    state.phi, state.p,
                    dt=self.dt,
                )
            elif ctrl_target.is_direct:
                servo_out = ServoOutput(
                    elevator=ctrl_target.elevator_direct or 0.0,
                    aileron =ctrl_target.aileron_direct  or 0.0,
                    rudder  =ctrl_target.rudder_direct   or 0.0,
                    throttle=ctrl_target.throttle_direct or 0.5,
                )

            # Convert normalised servo to radians (4 independent surfaces).
            # The rate controller outputs absolute deflection commands centred
            # on zero; we add the trim bias so the dynamics see the correct
            # total deflection (trim + control increment).
            da_left, da_right, dv_left, dv_right, throttle = servo_out.to_radians()
            ctrl_holder[0] = Controls(
                elevator=0.0,  # Virtual elevator (not used in V-tail)
                aileron =0.0,  # Virtual aileron (not used, using physical surfaces)
                rudder  =0.0,  # Virtual rudder (not used in V-tail)
                throttle=throttle,
                aileron_left =da_left,   # Physical left aileron
                aileron_right=da_right,  # Physical right aileron
                v_left  =dv_left + trim.de_trim,   # Add trim to V-tail
                v_right =dv_right + trim.de_trim,  # Add trim to V-tail
            )

            # Record
            des_pos = None
            if not use_trajectory and len(_circuit_wps) > 0:
                des_pos = _circuit_wps[_wp_idx].copy()  # current target WP (NED)
            elif traj_available and traj is not None:
                des_pos = traj.desired_state(t).pos
            history.record(
                t, state,
                elevator=servo_out.elevator,
                aileron =servo_out.aileron,
                rudder  =servo_out.rudder,
                throttle=servo_out.throttle,
                des_pos =des_pos,
            )

            # Step integrator
            try:
                integrator.step(self.dt)
            except RuntimeError as e:
                print(f"[Simulator] Integration error at t={t:.3f}: {e}")
                break

            t += self.dt

        history.trim()
        return SimulationResult(history, trim, self.aircraft_cfg.name, closed_loop)

    # ------------------------------------------------------------------

    def run_linear_analysis(
        self,
        pulses: Optional[List[Dict]] = None,
        duration: Optional[float] = None,
    ) -> LinearAnalysisResult:
        """
        Run 4-DOF linear (open-loop) analysis.

        Backward-compatible with project-1's FlightSimState.run_simulation().

        Parameters
        ----------
        pulses   : list of pulse dicts; if None, use a default 2-deg elevator pulse
        duration : simulation duration (s)

        Returns
        -------
        LinearAnalysisResult
        """
        if pulses is None:
            pulses = [{"start_time": 1.0, "duration": 0.5, "angle_deg": 2.0}]
        if duration is None:
            duration = self.duration

        model = LinearModel(self.params)
        return model.run_analysis(pulses, duration, uav_name=self.aircraft_cfg.name)

    # ------------------------------------------------------------------
    # Step-by-step API (for Reflex UI integration)
    # ------------------------------------------------------------------

    def init_step(self) -> AircraftSimState:
        """
        Initialise the step-by-step simulation.
        Must be called before the first call to step().
        """
        trim = self.dyn.compute_trim()
        self._trim = trim
        u0 = trim.U0 * np.cos(trim.alpha_trim)
        w0 = trim.U0 * np.sin(trim.alpha_trim)
        y0 = np.array([
            u0, 0.0, w0,
            0.0, 0.0, 0.0,
            0.0, trim.alpha_trim, 0.0,
            0.0, 0.0, -self.ap_params.ALT_HOLD_RTL,
        ])
        self._ctrl_state = Controls(v_left=trim.de_trim, v_right=trim.de_trim, throttle=0.5)

        def f_ode(t, y):
            ctrl = self._ctrl_state
            wind_ned = self.wind.get_wind_ned(t)
            phi, theta, psi = y[6], y[7], y[8]
            R = rotation_matrix_321(phi, theta, psi)
            wind_body = R.T @ wind_ned
            rho = compute_density(-y[11])
            return self.dyn.state_dot(t, y, ctrl, wind_body=wind_body, rho=rho)

        self._integrator = Dopri5Integrator(f_ode, y0, t0=0.0)
        return AircraftSimState.from_array(y0)

    def set_ardupilot_params(
        self,
        roll_attitude: dict = None,
        roll_rate: dict = None,
        pitch_attitude: dict = None,
        pitch_rate: dict = None,
        yaw_rate: dict = None
    ):
        """
        动态更新 ArduPilot 控制参数

        注意：此方法仅在使用 ArduPilot 控制器时有效。
        如果当前使用 PX4 控制器，将抛出异常。

        Args:
            roll_attitude: {'p': float} - 滚转姿态外环P增益
            roll_rate: {'p': float, 'i': float, 'd': float, 'ff': float} - 滚转速率内环增益
            pitch_attitude: {'p': float} - 俯仰姿态外环P增益
            pitch_rate: {'p': float, 'i': float, 'd': float, 'ff': float} - 俯仰速率内环增益
            yaw_rate: {'p': float, 'i': float, 'd': float} - 偏航速率内环增益

        Raises:
            RuntimeError: 如果当前使用的不是 ArduPilot 控制器
        """
        if self.controller_type != "ardupilot":
            raise RuntimeError(
                f"Cannot set ArduPilot parameters when using {self.controller_type} controller. "
                f"Use set_px4_params() instead, or create a new simulator with controller_type='ardupilot'."
            )

        # 更新姿态控制器参数
        if roll_attitude is not None:
            if 'p' in roll_attitude:
                self.ap_params.ROLL_P = roll_attitude['p']

        if pitch_attitude is not None:
            if 'p' in pitch_attitude:
                self.ap_params.PTCH_P = pitch_attitude['p']

        # 更新速率控制器参数
        if roll_rate is not None:
            if 'p' in roll_rate:
                self.ap_params.ROLL_RATE_P = roll_rate['p']
            if 'i' in roll_rate:
                self.ap_params.ROLL_RATE_I = roll_rate['i']
            if 'd' in roll_rate:
                self.ap_params.ROLL_RATE_D = roll_rate['d']
            if 'ff' in roll_rate:
                self.ap_params.ROLL_RATE_FF = roll_rate['ff']

        if pitch_rate is not None:
            if 'p' in pitch_rate:
                self.ap_params.PTCH_RATE_P = pitch_rate['p']
            if 'i' in pitch_rate:
                self.ap_params.PTCH_RATE_I = pitch_rate['i']
            if 'd' in pitch_rate:
                self.ap_params.PTCH_RATE_D = pitch_rate['d']
            if 'ff' in pitch_rate:
                self.ap_params.PTCH_RATE_FF = pitch_rate['ff']

        if yaw_rate is not None:
            if 'p' in yaw_rate:
                self.ap_params.YAW_RATE_P = yaw_rate['p']
            if 'i' in yaw_rate:
                self.ap_params.YAW_RATE_I = yaw_rate['i']
            if 'd' in yaw_rate:
                self.ap_params.YAW_RATE_D = yaw_rate['d']

        # 重新加载控制器增益
        self.att_ctrl.reload_gains(self.ap_params)
        self.rate_ctrl.reload_gains(self.ap_params)


    def set_px4_params(
        self,
        roll_attitude: dict = None,
        roll_rate: dict = None,
        pitch_attitude: dict = None,
        pitch_rate: dict = None,
        yaw_attitude: dict = None,
        yaw_rate: dict = None
    ):
        """
        动态更新 PX4 控制参数

        注意：此方法仅在使用 PX4 控制器时有效。
        如果当前使用 ArduPilot 控制器，将抛出异常。

        Args:
            roll_attitude: {'tc': float} - 滚转时间常数
            roll_rate: {'kp': float, 'ki': float, 'kff': float} - 滚转速率增益
            pitch_attitude: {'tc': float} - 俯仰时间常数
            pitch_rate: {'kp': float, 'ki': float, 'kff': float} - 俯仰速率增益
            yaw_attitude: {'tc': float} - 偏航时间常数
            yaw_rate: {'kp': float, 'ki': float, 'kff': float} - 偏航速率增益

        Raises:
            RuntimeError: 如果当前使用的不是 PX4 控制器
        """
        if self.controller_type != "px4":
            raise RuntimeError(
                f"Cannot set PX4 parameters when using {self.controller_type} controller. "
                f"Use set_ardupilot_params() instead, or create a new simulator with controller_type='px4'."
            )

        # 更新姿态控制器时间常数
        tc_dict = {}
        if roll_attitude is not None and 'tc' in roll_attitude:
            tc_dict['roll_tc'] = roll_attitude['tc']
        if pitch_attitude is not None and 'tc' in pitch_attitude:
            tc_dict['pitch_tc'] = pitch_attitude['tc']
        if yaw_attitude is not None and 'tc' in yaw_attitude:
            tc_dict['yaw_tc'] = yaw_attitude['tc']

        if tc_dict:
            self.att_ctrl.set_time_constants(**tc_dict)

        # 更新速率控制器增益（PX4 使用 kp, ki, kff）
        if roll_rate is not None:
            self.rate_ctrl.set_gains('roll', **roll_rate)
        if pitch_rate is not None:
            self.rate_ctrl.set_gains('pitch', **pitch_rate)
        if yaw_rate is not None:
            self.rate_ctrl.set_gains('yaw', **yaw_rate)




    def set_allocation_weights(self, weights: np.ndarray):
        """
        动态更新控制分配权重

        Args:
            weights: 舵面权重数组 (4,) [左副翼, 右副翼, 左V尾, 右V尾]
        """
        # 当前仿真器的 ServoMixer 可能需要扩展以支持权重调整
        # 这里提供接口，具体实现取决于 ServoMixer 的设计
        if hasattr(self.servo, 'set_weights'):
            self.servo.set_weights(weights)
        else:
            # 如果 ServoMixer 不支持权重设置，记录警告
            import logging
            logging.warning("ServoMixer does not support dynamic weight adjustment")


    def step(self, dt: Optional[float] = None) -> AircraftSimState:
        """
        Advance simulation by one time step.
        Returns updated AircraftSimState.
        """
        if not hasattr(self, "_integrator"):
            raise RuntimeError("Call init_step() before step()")
        if dt is None:
            dt = self.dt
        new_y = self._integrator.step(dt)
        return AircraftSimState.from_array(new_y)
