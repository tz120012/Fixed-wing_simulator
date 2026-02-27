"""
aerodynamics.py  –  Aerodynamic force and moment calculations.

All computations operate on the *body frame* using standard aerodynamic
sign conventions for a fixed-wing aircraft (positive-lift = upward force,
positive-moment = nose-up, etc.).

Reference: Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed.
"""

import numpy as np
from typing import Dict, Any
from utils.math_utils import angle_of_attack, sideslip_angle, dynamic_pressure


__all__ = ["AeroForces", "compute_aero_forces"]


class AeroForces:
    """Container for aerodynamic forces and moments in the body frame."""

    __slots__ = ("X", "Y", "Z", "L", "M", "N",
                 "CL", "CD", "CY", "Cl", "Cm", "Cn",
                 "alpha_rad", "beta_rad", "q_bar")

    def __init__(self):
        self.X = self.Y = self.Z = 0.0      # body-axis forces  (N)
        self.L = self.M = self.N = 0.0      # body-axis moments (N·m)
        self.CL = self.CD = self.CY = 0.0   # non-dimensional coefficients
        self.Cl = self.Cm = self.Cn = 0.0
        self.alpha_rad = self.beta_rad = 0.0
        self.q_bar = 0.0


def compute_aero_forces(
    u: float, v: float, w: float,
    p: float, q: float, r: float,
    de: float, da: float, dr: float,
    params: Dict[str, Any],
    wind_body: np.ndarray = None,
    rho: float = 1.225,
) -> AeroForces:
    """
    Compute aerodynamic forces and moments for a fixed-wing aircraft.

    Parameters
    ----------
    u, v, w   : body-frame velocity components (m/s)
    p, q, r   : body-frame angular rates (rad/s)
    de        : elevator deflection (rad, positive = trailing edge down)
    da        : aileron  deflection (rad, positive = right aileron down)
    dr        : rudder   deflection (rad, positive = trailing edge left)
    params    : aircraft parameter dict (from aircraft_database)
    wind_body : (3,) body-frame wind velocity [u_w,v_w,w_w] (m/s); None=no wind
    rho       : air density (kg/m³)

    Returns
    -------
    AeroForces instance
    """
    out = AeroForces()

    S    = params["S"]
    c    = params["c"]
    b    = params["b"]
    U0   = params["U0"]

    # --- True airspeed vector (subtract wind) ---------------------------------
    if wind_body is not None:
        u_a = u - wind_body[0]
        v_a = v - wind_body[1]
        w_a = w - wind_body[2]
    else:
        u_a, v_a, w_a = u, v, w

    airspeed = max(np.sqrt(u_a**2 + v_a**2 + w_a**2), 1.0)
    out.q_bar = dynamic_pressure(rho, airspeed)

    # --- Aerodynamic angles ---------------------------------------------------
    alpha = angle_of_attack(u_a, w_a)          # rad
    beta  = sideslip_angle(v_a, airspeed)       # rad
    out.alpha_rad = alpha
    out.beta_rad  = beta

    # --- Non-dimensional angular rates ----------------------------------------
    p_hat = p * b  / (2.0 * U0)
    q_hat = q * c  / (2.0 * U0)
    r_hat = r * b  / (2.0 * U0)

    # --- Longitudinal coefficients -------------------------------------------
    CL = (params["CL_0"]
          + params["CL_alpha"] * alpha
          + params["CL_q"]    * q_hat
          + params["CL_deltae"] * de)

    CD = (params["CD_0"]
          + params["CD_alpha"] * alpha
          + params["CD_q"]     * q_hat
          + params["CD_deltae"] * de)

    Cm = (params["Cm_0"]
          + params["Cm_alpha"] * alpha
          + params["Cm_q"]    * q_hat
          + params["Cm_deltae"] * de)

    # --- Lateral-directional coefficients ------------------------------------
    CY = (params["CYb"] * beta
          + params["CYp"] * p_hat
          + params["CYr"] * r_hat
          + params["CYda"] * da
          + params["CYdr"] * dr)

    Cl = (params["Clb"] * beta
          + params["Clp"] * p_hat
          + params["Clr"] * r_hat
          + params["Clda"] * da
          + params["Cldr"] * dr)

    Cn = (params["Cnb"] * beta
          + params["Cnp"] * p_hat
          + params["Cnr"] * r_hat
          + params["Cnda"] * da
          + params["Cndr"] * dr)

    out.CL, out.CD, out.CY = CL, CD, CY
    out.Cl, out.Cm, out.Cn = Cl, Cm, Cn

    q_bar = out.q_bar

    # --- Forces in body frame ------------------------------------------------
    # Lift acts perpendicular to airspeed vector (approximated as -Z_body)
    # Drag acts opposite to airspeed vector (≈ -X_body)
    # The 2-D rotation from wind axes to body axes:
    #   X_body = -CD*cos(alpha) + CL*sin(alpha)  (approx: X = -CD at small alpha)
    #   Z_body = -CL*cos(alpha) - CD*sin(alpha)  (approx: Z = -CL at small alpha)
    # For generality we use the exact wind-axis to body-axis transform:
    ca, sa = np.cos(alpha), np.sin(alpha)
    out.X = q_bar * S * (-CD * ca + CL * sa)
    out.Y = q_bar * S * CY
    out.Z = q_bar * S * (-CL * ca - CD * sa)

    # --- Moments in body frame -----------------------------------------------
    out.L = q_bar * S * b * Cl
    out.M = q_bar * S * c * Cm
    out.N = q_bar * S * b * Cn

    return out
