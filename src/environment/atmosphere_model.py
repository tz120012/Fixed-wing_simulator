"""
atmosphere_model.py  –  International Standard Atmosphere (ISA) model.

Covers troposphere (0–11 km) and lower stratosphere (11–20 km).
Physical constants match project-1 constants.py.
"""

import numpy as np

# Constants
G0    = 9.80665    # m/s²
R_GAS = 287.05     # J/(kg·K)
GAMMA = 1.4
T0    = 288.15     # K  (sea-level standard temperature)
P0    = 101325.0   # Pa (sea-level standard pressure)
RHO0  = 1.225      # kg/m³ (sea-level standard density)
L_TROP = -0.0065   # K/m lapse rate (troposphere)
H_TROP = 11000.0   # m  tropopause altitude
T_TROP = T0 + L_TROP * H_TROP  # 216.65 K
P_TROP = P0 * (T_TROP / T0) ** (-G0 / (L_TROP * R_GAS))
RHO_TROP = P_TROP / (R_GAS * T_TROP)


def compute_temperature(altitude_m: float) -> float:
    """
    ISA temperature at *altitude_m* (m, positive up).

    Returns T in Kelvin.
    """
    alt = np.clip(float(altitude_m), -500.0, 80000.0)
    if alt <= H_TROP:
        return T0 + L_TROP * alt
    else:
        return T_TROP  # isothermal stratosphere (up to 20 km)


def compute_pressure(altitude_m: float) -> float:
    """ISA pressure (Pa) at *altitude_m* (m)."""
    alt = float(altitude_m)
    if alt <= H_TROP:
        T = compute_temperature(alt)
        return P0 * (T / T0) ** (-G0 / (L_TROP * R_GAS))
    else:
        dh = alt - H_TROP
        return P_TROP * np.exp(-G0 * dh / (R_GAS * T_TROP))


def compute_density(altitude_m: float) -> float:
    """ISA air density ρ (kg/m³) at *altitude_m* (m)."""
    T = compute_temperature(altitude_m)
    P = compute_pressure(altitude_m)
    return P / (R_GAS * T)


def compute_speed_of_sound(altitude_m: float) -> float:
    """ISA speed of sound (m/s) at *altitude_m*."""
    T = compute_temperature(altitude_m)
    return np.sqrt(GAMMA * R_GAS * T)


def atmosphere(altitude_m: float):
    """
    Convenience function returning (rho, P, T, a) at *altitude_m*.

    Returns
    -------
    rho : density    (kg/m³)
    P   : pressure   (Pa)
    T   : temperature (K)
    a   : speed of sound (m/s)
    """
    T   = compute_temperature(altitude_m)
    P   = compute_pressure(altitude_m)
    rho = P / (R_GAS * T)
    a   = np.sqrt(GAMMA * R_GAS * T)
    return rho, P, T, a
