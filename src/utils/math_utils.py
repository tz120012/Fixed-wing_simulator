"""
math_utils.py  –  General mathematical utility functions.
All functions are NumPy-only (no additional dependencies).
"""

import numpy as np


# ---------------------------------------------------------------------------
# Angle utilities
# ---------------------------------------------------------------------------

def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi] (radians)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def wrap_angle_deg(angle: float) -> float:
    """Wrap angle to [-180, 180] (degrees)."""
    return (angle + 180.0) % 360.0 - 180.0


def saturate(value: float, v_min: float, v_max: float) -> float:
    """Clamp *value* to [v_min, v_max]."""
    return float(np.clip(value, v_min, v_max))


def deg2rad(deg):
    """Vectorised degrees → radians."""
    return np.asarray(deg, dtype=float) * (np.pi / 180.0)


def rad2deg(rad):
    """Vectorised radians → degrees."""
    return np.asarray(rad, dtype=float) * (180.0 / np.pi)


# ---------------------------------------------------------------------------
# Rotation matrices (ZYX / 3-2-1 Euler convention, NED frame)
# phi = roll, theta = pitch, psi = yaw
# ---------------------------------------------------------------------------

def rotation_matrix_321(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Body-to-NED direction cosine matrix (DCM) using 3-2-1 Euler angles.

    Parameters
    ----------
    phi   : roll  angle (rad)
    theta : pitch angle (rad)
    psi   : yaw   angle (rad)

    Returns
    -------
    R : (3, 3) ndarray  –  R @ v_body = v_NED
    """
    cp, sp = np.cos(phi),   np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cs, ss = np.cos(psi),   np.sin(psi)

    R = np.array([
        [ct * cs,  sp * st * cs - cp * ss,  cp * st * cs + sp * ss],
        [ct * ss,  sp * st * ss + cp * cs,  cp * st * ss - sp * cs],
        [-st,      sp * ct,                  cp * ct              ],
    ])
    return R


def body_to_ned(v_body: np.ndarray, phi: float, theta: float, psi: float) -> np.ndarray:
    """Transform a vector from body frame to NED frame."""
    return rotation_matrix_321(phi, theta, psi) @ v_body


def ned_to_body(v_ned: np.ndarray, phi: float, theta: float, psi: float) -> np.ndarray:
    """Transform a vector from NED frame to body frame (R^T)."""
    return rotation_matrix_321(phi, theta, psi).T @ v_ned


def euler_rates(p: float, q: float, r: float,
                phi: float, theta: float) -> np.ndarray:
    """
    Euler angle rates from body angular rates.

    Returns [phi_dot, theta_dot, psi_dot].
    Singular at theta = ±90 deg; numerical protection via small ε.
    """
    eps = 1e-9
    cos_theta = np.cos(theta)
    if abs(cos_theta) < eps:
        cos_theta = np.sign(cos_theta) * eps

    tan_theta = np.sin(theta) / cos_theta
    sin_phi   = np.sin(phi)
    cos_phi   = np.cos(phi)

    phi_dot   = p + sin_phi * tan_theta * q + cos_phi * tan_theta * r
    theta_dot = cos_phi * q - sin_phi * r
    psi_dot   = (sin_phi / cos_theta) * q + (cos_phi / cos_theta) * r

    return np.array([phi_dot, theta_dot, psi_dot])


# ---------------------------------------------------------------------------
# Aerodynamics helpers
# ---------------------------------------------------------------------------

def angle_of_attack(u: float, w: float) -> float:
    """Angle of attack alpha = arctan(w / u) in radians."""
    return np.arctan2(w, u)


def sideslip_angle(v: float, airspeed: float) -> float:
    """
    Sideslip angle beta = arcsin(v / V) in radians.
    *airspeed* must be > 0; numerical clamp applied.
    """
    V = max(airspeed, 1e-3)
    return np.arcsin(np.clip(v / V, -1.0, 1.0))


def dynamic_pressure(rho: float, airspeed: float) -> float:
    """q_bar = 0.5 * rho * V^2."""
    return 0.5 * rho * airspeed ** 2
