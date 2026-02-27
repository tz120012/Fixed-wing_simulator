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

        plotter.plot_6dof_matplotlib(h, self.uav_name)
        animator.animate(h, self.uav_name, show=show)


# ---------------------------------------------------------------------------
# Main simulator class
# ---------------------------------------------------------------------------

class FixedWingSimulator:
    """
    Fixed-wing UAV simulator with ArduPilot-compatible control.

    Parameters
    ----------
    aircraft_name  : aircraft key from the database
    config_dir     : path to config/ directory
    dt             : simulation time step (s)
    duration       : total simulation duration (s)
    initial_mode   : starting FlightMode (string or FlightMode enum)
    wind_type      : 'NONE' | 'FIXED' | 'SINE' | 'RANDOMSINE'
    traj_type      : 'minimum_snap' | 'minimum_jerk'
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

        # --- Aircraft -------------------------------------------------------
        self.aircraft_cfg: AircraftConfig = AircraftFactory.create(aircraft_name)
        self.params = self.aircraft_cfg.aero_params

        # --- Environment ----------------------------------------------------
        env_wind_type  = wind_type or sim_cfg.get("wind_type", "NONE")
        wind_speed     = sim_cfg.get("wind_speed", 5.0)
        wind_dir_deg   = sim_cfg.get("wind_direction_deg", 270.0)
        self.wind = Wind(env_wind_type, speed=wind_speed, direction_deg=wind_dir_deg)

        # --- Control parameters (ArduPilot) ---------------------------------
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
        self.nav_ctrl  = NavigationController(
            l1_period=self.ap_params.NAVL1_PERIOD,
            l1_damping=self.ap_params.NAVL1_DAMPING,
            max_roll=np.radians(self.ap_params.LIM_ROLL_DEG),
            max_pitch=np.radians(self.ap_params.LIM_PITCH_MAX),
            min_pitch=np.radians(self.ap_params.LIM_PITCH_MIN),
            cruise_speed=self.ap_params.AIRSPEED_CRUISE,
            cruise_alt=self.ap_params.ALT_HOLD_RTL,
        )
        self.att_ctrl  = AttitudeController(self.ap_params, dt=dt)
        self.rate_ctrl = RateController(self.ap_params, dt=dt)
        self.servo     = ServoMixer(self.ap_params, dt=dt)

        # --- Trajectory / planning ------------------------------------------
        self.wp_mgr = WaypointManager(
            average_speed=self.ap_params.AIRSPEED_CRUISE,
            traj_type=traj_type,
            yaw_mode="yaw_follow",
        )
        traj_path = os.path.join(config_dir, "trajectory.yaml")
        if os.path.isfile(traj_path):
            self.wp_mgr.load_from_yaml(traj_path)

        # --- Dynamics -------------------------------------------------------
        self.dyn = NonlinearModel(self.params)
        self._trim: Optional[TrimResult] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self, closed_loop: bool = True) -> SimulationResult:
        """
        Run the complete simulation.

        Parameters
        ----------
        closed_loop : if True, use the 5-layer ArduPilot control system;
                      if False, run open-loop (trim-hold) dynamics only.

        Returns
        -------
        SimulationResult
        """
        trim  = self.dyn.compute_trim()
        self._trim = trim
        n_steps = int(self.duration / self.dt) + 1
        history = StateHistory(n_steps)

        # Initial state
        u0 = trim.U0 * np.cos(trim.alpha_trim)
        w0 = trim.U0 * np.sin(trim.alpha_trim)
        y0 = np.array([
            u0, 0.0, w0,
            0.0, 0.0, 0.0,
            0.0, trim.alpha_trim, 0.0,
            0.0, 0.0, -self.ap_params.ALT_HOLD_RTL,
        ])

        # Build a dynamic ODE function that references the control system
        ctrl_holder = [Controls(elevator=trim.de_trim, throttle=0.5)]

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

        # Ensure trajectory is built
        if len(self.wp_mgr._waypoints_ned) >= 2:
            traj = self.wp_mgr.trajectory
            traj_available = True
        else:
            traj_available = False

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

            if closed_loop and traj_available:
                des = traj.desired_state(t)

                # Build path segment toward desired position
                seg = PathSegment(
                    start=state.pos_ned,
                    end=des.pos,
                    target_speed=self.ap_params.AIRSPEED_CRUISE,
                )
                nav_target = self.nav_ctrl.update(ac_state, seg, dt=self.dt)

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

            # Convert normalised servo to radians
            de, da, dr = servo_out.to_radians()
            ctrl_holder[0] = Controls(
                elevator=de,
                aileron =da,
                rudder  =dr,
                throttle=servo_out.throttle,
            )

            # Record
            des_pos = None
            if traj_available:
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
        self._ctrl_state = Controls(elevator=trim.de_trim, throttle=0.5)

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
