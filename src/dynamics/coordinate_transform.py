"""
coordinate_transform.py  –  Coordinate frame transformations.

Convention: NED (North-East-Down) body frame with 3-2-1 Euler angles.
  phi   = roll  (rotation about x-body)
  theta = pitch (rotation about y-body)
  psi   = yaw   (rotation about z-body)
"""

import numpy as np
from utils.math_utils import (
    rotation_matrix_321,
    body_to_ned,
    ned_to_body,
    euler_rates,
)


__all__ = [
    "body_to_ned",
    "ned_to_body",
    "euler_rates",
    "dcm_from_euler",
    "wind_to_body_frame",
    "airspeed_vector",
]


def dcm_from_euler(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Direction cosine matrix (DCM) from Euler angles.
    Equivalent to rotation_matrix_321; provided as a named alias.

    Returns R such that  v_NED = R @ v_body.
    """
    return rotation_matrix_321(phi, theta, psi)


def wind_to_body_frame(wind_ned: np.ndarray,
                       phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Convert a NED wind vector to the aircraft body frame.

    Parameters
    ----------
    wind_ned : (3,) array  [v_north, v_east, v_down]  (m/s)
    phi, theta, psi : Euler angles (rad)

    Returns
    -------
    wind_body : (3,) array  [u_w, v_w, w_w]  (m/s) in body frame
    """
    return ned_to_body(wind_ned, phi, theta, psi)


def airspeed_vector(vel_body: np.ndarray, wind_body: np.ndarray) -> np.ndarray:
    """
    True airspeed vector = body velocity – wind in body frame.

    Parameters
    ----------
    vel_body  : (3,) body-frame ground velocity   [u, v, w]  (m/s)
    wind_body : (3,) body-frame wind velocity      [u_w, v_w, w_w] (m/s)

    Returns
    -------
    v_air : (3,) airspeed vector  (m/s)
    """
    return vel_body - wind_body
