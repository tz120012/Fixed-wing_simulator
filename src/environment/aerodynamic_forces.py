"""
aerodynamic_forces.py  –  Additional aerodynamic drag due to wind.

Used to compute the *incremental* body-frame forces caused by wind relative
to the current aircraft velocity (beyond what is already captured in the
baseline aerodynamics computation).
"""

import numpy as np
from typing import Dict, Any


def compute_wind_drag_forces(
    wind_body: np.ndarray,
    state_uvw: np.ndarray,
    params: Dict[str, Any],
    rho: float = 1.225,
) -> np.ndarray:
    """
    Compute incremental body-frame drag forces due to wind.

    The function estimates the aerodynamic drag increment caused by the
    relative wind speed using a simple drag model:
        ΔF = 0.5 * rho * S * CD_0 * |v_rel| * v_rel_body

    This is suitable for perturbation / sensitivity analysis.

    Parameters
    ----------
    wind_body  : (3,) body-frame wind velocity [u_w, v_w, w_w] (m/s)
    state_uvw  : (3,) body-frame aircraft velocity [u, v, w]   (m/s)
    params     : aircraft parameter dict
    rho        : air density (kg/m³)

    Returns
    -------
    dF : (3,) incremental force in body frame (N)  [dX, dY, dZ]
    """
    v_rel  = state_uvw - wind_body          # relative airspeed in body frame
    V_rel  = np.linalg.norm(v_rel)

    if V_rel < 1e-3:
        return np.zeros(3)

    S    = params["S"]
    CD0  = params["CD_0"]

    q_bar_rel = 0.5 * rho * V_rel**2
    # Drag opposes relative motion
    F_drag_mag = q_bar_rel * S * CD0
    dF = -F_drag_mag * (v_rel / V_rel)

    return dF
