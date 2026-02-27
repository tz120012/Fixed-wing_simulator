"""
nonlinear_model.py  –  6-DOF full nonlinear equations of motion.

State vector (12-D, NED frame):
  [0]  u     – forward body velocity (m/s)
  [1]  v     – lateral body velocity (m/s)
  [2]  w     – vertical body velocity (m/s)
  [3]  p     – roll rate (rad/s)
  [4]  q     – pitch rate (rad/s)
  [5]  r     – yaw rate (rad/s)
  [6]  phi   – roll angle (rad)
  [7]  theta – pitch angle (rad)
  [8]  psi   – yaw angle (rad)
  [9]  x_N   – north position (m)
  [10] x_E   – east  position (m)
  [11] x_D   – down  position (m, positive down in NED)

Reference:
  Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed.
"""

import numpy as np
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field
from scipy.integrate import solve_ivp

from dynamics.aerodynamics import compute_aero_forces
from utils.math_utils import rotation_matrix_321, euler_rates, dynamic_pressure

G     = 9.80665
RHO0  = 1.225
R_GAS = 287.05
GAMMA = 1.4
A_SOUND = np.sqrt(GAMMA * R_GAS * 288.15)


@dataclass
class Controls:
    """Aircraft control surface deflections and throttle."""
    elevator: float = 0.0   # rad, positive = trailing-edge down
    aileron:  float = 0.0   # rad, positive = right aileron down
    rudder:   float = 0.0   # rad, positive = trailing-edge left
    throttle: float = 0.0   # 0–1 (normalised)


@dataclass
class TrimResult:
    alpha_trim: float  # rad
    de_trim:    float  # rad
    U0:         float  # m/s


@dataclass
class NonlinearSimResult:
    t:          np.ndarray              # (N,) s
    y:          np.ndarray              # (12, N)
    controls:   Dict[str, np.ndarray]   # {"elevator","aileron","rudder","throttle"}
    derived:    Dict[str, np.ndarray]   # alpha, beta, airspeed, kinetic, potential
    trim:       TrimResult
    uav_name:   str

    def summary(self) -> str:
        lines = [
            f"=== {self.uav_name} 6-DOF Nonlinear Simulation ===",
            f"Trim speed U0 = {self.trim.U0:.2f} m/s  "
            f"({self.trim.U0 * 1.94384:.2f} kn)",
            f"α_trim = {np.degrees(self.trim.alpha_trim):.3f} deg  |  "
            f"δe_trim = {np.degrees(self.trim.de_trim):.3f} deg",
            f"Duration: {self.t[-1]:.1f} s  |  Steps: {len(self.t)}",
        ]
        return "\n".join(lines)

    def plot(self):
        """Quick Matplotlib overview plot (standalone use)."""
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(4, 3, figsize=(16, 12))
        fig.suptitle(f"{self.uav_name} 6-DOF Response", fontsize=14)

        rows = [
            ("u (m/s)",       self.y[0]),
            ("v (m/s)",       self.y[1]),
            ("w (m/s)",       self.y[2]),
            ("p (deg/s)",     np.degrees(self.y[3])),
            ("q (deg/s)",     np.degrees(self.y[4])),
            ("r (deg/s)",     np.degrees(self.y[5])),
            ("φ (deg)",       np.degrees(self.y[6])),
            ("θ (deg)",       np.degrees(self.y[7])),
            ("ψ (deg)",       np.degrees(self.y[8])),
            ("North (m)",     self.y[9]),
            ("East (m)",      self.y[10]),
            ("Alt (m)",      -self.y[11]),
        ]
        for ax, (lbl, data) in zip(axes.flat, rows):
            ax.plot(self.t, data, linewidth=1.2)
            ax.set_title(lbl, fontsize=9)
            ax.set_xlabel("t (s)", fontsize=8)
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


class NonlinearModel:
    """
    6-DOF nonlinear equations of motion for a fixed-wing aircraft.

    Provides:
      compute_trim() – solve for level-flight trim condition
      state_dot()    – evaluate time derivative of 12-D state vector
      simulate()     – integrate ODE over a given time horizon
    """

    def __init__(self, params: Dict[str, Any]):
        """
        Parameters
        ----------
        params : aircraft parameter dict from aircraft_database
        """
        self.params = params
        U0 = params["Mach"] * A_SOUND
        # Pre-compute derived params used in every call to state_dot
        self._p = dict(params)
        self._p["U0"]    = U0
        self._p["rho"]   = RHO0
        self._p["q_bar"] = dynamic_pressure(RHO0, U0)

    # ------------------------------------------------------------------
    # Trim solver
    # ------------------------------------------------------------------

    def compute_trim(self) -> TrimResult:
        """
        Solve for (alpha_trim, de_trim) for level wings-level flight.

        Returns TrimResult with alpha_trim, de_trim (rad) and U0 (m/s).
        """
        p     = self._p
        q_bar = p["q_bar"]
        S, m  = p["S"], p["mass"]

        CL0     = p["CL_0"];      CL_a = p["CL_alpha"]; CL_de = p["CL_deltae"]
        Cm0     = p["Cm_0"];      Cm_a = p["Cm_alpha"]; Cm_de = p["Cm_deltae"]

        CL_req = m * G / (q_bar * S)
        A  = np.array([[CL_a, CL_de], [Cm_a, Cm_de]])
        b  = np.array([CL_req - CL0, -Cm0])
        try:
            alpha_t, de_t = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            alpha_t = (CL_req - CL0) / max(CL_a, 1e-9)
            de_t    = 0.0

        return TrimResult(alpha_t, de_t, p["U0"])

    # ------------------------------------------------------------------
    # Equations of motion
    # ------------------------------------------------------------------

    def state_dot(
        self,
        t: float,
        state: np.ndarray,
        controls: Controls,
        wind_body: Optional[np.ndarray] = None,
        rho: float = RHO0,
    ) -> np.ndarray:
        """
        Compute 12-D state derivative at time *t*.

        Parameters
        ----------
        t         : current time (s)  – not used explicitly (autonomous)
        state     : (12,) current state vector
        controls  : Controls instance
        wind_body : (3,) wind in body frame (m/s); None → no wind
        rho       : air density (kg/m³)

        Returns
        -------
        dxdt : (12,) state derivative vector
        """
        p   = self._p
        m   = p["mass"]
        Ixx = p["ixx"]
        Iyy = p["Iyy"]
        Izz = p["izz"]
        Ixz = p["ixz"]

        u, v, w       = state[0], state[1], state[2]
        pp, q, r      = state[3], state[4], state[5]
        phi, theta, psi = state[6], state[7], state[8]

        de = controls.elevator
        da = controls.aileron
        dr = controls.rudder
        throttle = controls.throttle

        # --- Aerodynamic forces & moments ------------------------------------
        aero = compute_aero_forces(
            u, v, w, pp, q, r, de, da, dr, p,
            wind_body=wind_body, rho=rho,
        )

        # --- Thrust (simple proportional model) ------------------------------
        # T = throttle * T_max
        # T_max based on realistic thrust-to-weight ratio (~0.20 for medium UAV)
        # TB2 @ 40m/s: D≈494N, T_max=1372N → thr_cruise≈0.36, max_climb≈5.2m/s
        T_max = m * G * 0.20   # TWR=0.20: balanced for climb + cruise
        thrust = throttle * T_max

        # --- Gravity in body frame (NED: z positive down) -------------------
        sp, cp = np.sin(phi),   np.cos(phi)
        st, ct = np.sin(theta), np.cos(theta)
        Xg = -m * G * st
        Yg =  m * G * sp * ct
        Zg =  m * G * cp * ct

        # --- Total forces in body frame --------------------------------------
        X_tot = aero.X + thrust + Xg
        Y_tot = aero.Y + Yg
        Z_tot = aero.Z + Zg

        # --- Translational accelerations (body frame) -----------------------
        u_dot = r * v  - q * w  + X_tot / m
        v_dot = pp * w - r * u  + Y_tot / m
        w_dot = q * u  - pp * v + Z_tot / m

        # --- Rotational equations (Euler with inertia coupling) -------------
        denom = Ixx * Izz - Ixz**2
        L_aero, M_aero, N_aero = aero.L, aero.M, aero.N

        p_dot = (Izz * L_aero + Ixz * N_aero
                 - (Ixz * (Iyy - Ixx - Izz) * pp * r
                    + (Ixz**2 + Izz * (Izz - Iyy)) * q * r)) / denom
        q_dot = M_aero / Iyy
        r_dot = (Ixx * N_aero + Ixz * L_aero
                 + (Ixz * (Iyy - Ixx - Izz) * pp * q
                    + (Ixz**2 + Ixx * (Ixx - Iyy)) * q * r)) / denom

        # --- Euler angle kinematics -----------------------------------------
        euler_d = euler_rates(pp, q, r, phi, theta)
        phi_dot, theta_dot, psi_dot = euler_d

        # --- Position kinematics (body → NED) --------------------------------
        R = rotation_matrix_321(phi, theta, psi)
        vel_ned = R @ np.array([u, v, w])
        xN_dot, xE_dot, xD_dot = vel_ned

        return np.array([
            u_dot, v_dot, w_dot,
            p_dot, q_dot, r_dot,
            phi_dot, theta_dot, psi_dot,
            xN_dot, xE_dot, xD_dot,
        ])

    # ------------------------------------------------------------------
    # ODE wrapper (for use with scipy dopri5 in real-time loop)
    # ------------------------------------------------------------------

    def make_ode_func(self, get_controls, get_wind=None, get_rho=None):
        """
        Return a callable f(t, y) suitable for scipy.integrate.ode (dopri5).

        Parameters
        ----------
        get_controls : callable(t) -> Controls
        get_wind     : callable(t) -> (3,) NED wind OR None
        get_rho      : callable(t, alt_m) -> rho  OR None (uses constant RHO0)
        """
        def f(t, y):
            ctrl = get_controls(t)
            wind_ned = get_wind(t) if get_wind else None
            wind_body = None
            if wind_ned is not None:
                phi, theta, psi = y[6], y[7], y[8]
                R = rotation_matrix_321(phi, theta, psi)
                wind_body = R.T @ wind_ned
            rho = get_rho(t, -y[11]) if get_rho else RHO0
            return self.state_dot(t, y, ctrl, wind_body=wind_body, rho=rho)
        return f

    # ------------------------------------------------------------------
    # Batch simulation (solve_ivp)
    # ------------------------------------------------------------------

    def simulate(
        self,
        pulses: List[Dict],
        duration: float = 10.0,
        n_points: int = 500,
        wind_func=None,
    ) -> NonlinearSimResult:
        """
        Simulate 6-DOF open-loop response to control pulses.

        Parameters
        ----------
        pulses   : list of dicts with keys:
                     start_time, duration, angle_deg (elevator),
                     roll_deg (aileron), yaw_deg (rudder), throttle
        duration : (s)
        n_points : time evaluation points
        wind_func: callable(t) -> (3,) NED wind, or None

        Returns
        -------
        NonlinearSimResult
        """
        trim = self.compute_trim()

        def get_controls(t: float) -> Controls:
            de_cmd = da_cmd = dr_cmd = 0.0
            thr = 1.0
            count = 0
            for p in pulses:
                if p["start_time"] <= t <= p["start_time"] + p["duration"]:
                    da_cmd +=  np.deg2rad(p.get("roll_deg", 0.0))
                    de_cmd += -np.deg2rad(p.get("angle_deg", 0.0))
                    dr_cmd +=  np.deg2rad(p.get("yaw_deg", 0.0))
                    thr    += p.get("throttle", 1.0)
                    count  += 1
            if count > 1:
                thr /= count
            return Controls(
                elevator=trim.de_trim + de_cmd,
                aileron=da_cmd,
                rudder=dr_cmd,
                throttle=thr,
            )

        def get_wind(t: float):
            return wind_func(t) if wind_func else None

        f_ode = self.make_ode_func(get_controls, get_wind)

        # Initial trimmed state
        u0 = trim.U0 * np.cos(trim.alpha_trim)
        w0 = trim.U0 * np.sin(trim.alpha_trim)
        y0 = np.array([
            u0, 0.0, w0,
            0.0, 0.0, 0.0,
            0.0, trim.alpha_trim, 0.0,
            0.0, 0.0, 0.0,
        ])

        t_eval = np.linspace(0, duration, n_points)
        sol = solve_ivp(
            f_ode, [0, duration], y0,
            t_eval=t_eval, rtol=1e-6, atol=1e-6, max_step=0.1,
        )

        # Build control histories
        ctrl_hist = {"elevator": [], "aileron": [], "rudder": [], "throttle": []}
        for tt in sol.t:
            c = get_controls(tt)
            ctrl_hist["elevator"].append(c.elevator)
            ctrl_hist["aileron"].append(c.aileron)
            ctrl_hist["rudder"].append(c.rudder)
            ctrl_hist["throttle"].append(c.throttle)
        ctrl_hist = {k: np.array(v) for k, v in ctrl_hist.items()}

        # Derived quantities
        u_h, v_h, w_h = sol.y[0], sol.y[1], sol.y[2]
        airspeed = np.maximum(np.sqrt(u_h**2 + v_h**2 + w_h**2), 1e-3)
        alpha    = np.degrees(np.arctan2(w_h, u_h))
        beta     = np.degrees(np.arcsin(np.clip(v_h / airspeed, -1.0, 1.0)))
        m_v      = self.params["mass"]
        kinetic  = 0.5 * m_v * airspeed**2
        potential = m_v * G * (-sol.y[11])  # altitude = -z_D

        return NonlinearSimResult(
            t=sol.t,
            y=sol.y,
            controls=ctrl_hist,
            derived={
                "alpha":     alpha,
                "beta":      beta,
                "airspeed":  airspeed,
                "kinetic":   kinetic,
                "potential": potential,
            },
            trim=trim,
            uav_name=self.params.get("name", "UAV"),
        )
