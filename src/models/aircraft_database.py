"""
aircraft_database.py  –  Complete aircraft parameter database.

All 7 aircraft from the original project-1 (flight_dynamics_simulator_ui_design)
are reproduced here verbatim, plus additional derived/computed fields required
by the new 6-DOF engine and ArduPilot-compatible control layer.

Parameter naming follows project-1 UAVParameter TypedDict exactly so that the
new library is a drop-in replacement for the original data source.
"""

from typing import Dict, Any, List

# Physical / atmospheric constants
G       = 9.80665   # m/s²
RHO0    = 1.225     # kg/m³  (sea-level ISA)
R_GAS   = 287.05    # J/(kg·K)
GAMMA   = 1.4
import math
A_SOUND = math.sqrt(GAMMA * R_GAS * 288.15)  # ≈ 340.3 m/s


# ---------------------------------------------------------------------------
# Main database
# Each entry follows the UAVParameter TypedDict from project-1 EXACTLY,
# extended with the "name" key (for display) and "ardupilot_defaults" sub-dict.
# ---------------------------------------------------------------------------

_DB: Dict[str, Dict[str, Any]] = {

    "TB2": {
        # Identification
        "name": "TB2",
        "company": "Baykar",
        "country": "Turkey",
        # Geometry / inertia
        "mass": 700.0,   "S": 9.34,   "c": 0.78,  "b": 12.0,
        "Iyy": 2500.0,   "ixx": 3500, "izz": 5000, "ixz": 100,
        "Mach": 0.12,
        # Longitudinal aero
        "CL_0": 0.3,   "CL_alpha": 5.5,  "CL_q": 5.0,  "CL_deltae": 0.4,  "CL_u": 0.0,
        "CD_0": 0.05,  "CD_alpha": 0.3,  "CD_q": 0.0,  "CD_deltae": 0.0,  "CD_u": 0.0,
        "Cm_0": 0.0,   "Cm_alpha": -0.6, "Cm_q": -5.0, "Cm_deltae": -1.0, "Cm_u": 0.0,
        # Lateral-directional aero
        "CYb": -0.5, "CYp": 0.0, "CYr": 0.0,  "CYda": 0.0,  "CYdr": 0.1,
        "Clb": -0.05,"Clp": -0.4,"Clr": 0.0,  "Clda": 0.2,  "Cldr": 0.01,
        "Cnb":  0.05,"Cnp":  0.0,"Cnr": -0.08,"Cnda": 0.0,  "Cndr": -0.07,
    },

    "Anka": {
        "name": "Anka",
        "company": "TUSAŞ", "country": "Turkey",
        "mass": 1700.0,  "S": 18.0,  "c": 1.05, "b": 17.5,
        "Iyy": 6000.0,   "ixx": 8000,"izz": 12000,"ixz": 200,
        "Mach": 0.18,
        "CL_0": 0.35,  "CL_alpha": 5.7,  "CL_q": 5.5,  "CL_deltae": 0.5,  "CL_u": 0.0,
        "CD_0": 0.06,  "CD_alpha": 0.32, "CD_q": 0.0,  "CD_deltae": 0.0,  "CD_u": 0.0,
        "Cm_0": 0.0,   "Cm_alpha": -0.8, "Cm_q": -6.0, "Cm_deltae": -1.2, "Cm_u": 0.0,
        "CYb": -0.5,"CYp": 0.0,"CYr": 0.0, "CYda": 0.0,"CYdr": 0.1,
        "Clb": -0.05,"Clp": -0.4,"Clr": 0.0,"Clda": 0.2,"Cldr": 0.01,
        "Cnb":  0.05,"Cnp":  0.0,"Cnr": -0.08,"Cnda": 0.0,"Cndr": -0.07,
    },

    "Aksungur": {
        "name": "Aksungur",
        "company": "TUSAŞ", "country": "Turkey",
        "mass": 3300.0,  "S": 30.0,  "c": 1.25, "b": 24.2,
        "Iyy": 12000.0,  "ixx": 15000,"izz": 25000,"ixz": 500,
        "Mach": 0.21,
        "CL_0": 0.25,  "CL_alpha": 5.2,  "CL_q": 6.0,  "CL_deltae": 0.6,  "CL_u": 0.0,
        "CD_0": 0.07,  "CD_alpha": 0.35, "CD_q": 0.0,  "CD_deltae": 0.0,  "CD_u": 0.0,
        "Cm_0": 0.0,   "Cm_alpha": -0.7, "Cm_q": -7.0, "Cm_deltae": -1.5, "Cm_u": 0.0,
        "CYb": -0.5,"CYp": 0.0,"CYr": 0.0, "CYda": 0.0,"CYdr": 0.1,
        "Clb": -0.05,"Clp": -0.4,"Clr": 0.0,"Clda": 0.2,"Cldr": 0.01,
        "Cnb":  0.05,"Cnp":  0.0,"Cnr": -0.08,"Cnda": 0.0,"Cndr": -0.07,
    },

    "Karayel": {
        "name": "Karayel",
        "company": "Vestel", "country": "Turkey",
        "mass": 630.0,   "S": 9.0,   "c": 0.75, "b": 13.0,
        "Iyy": 2000.0,   "ixx": 2800,"izz": 4500,"ixz": 90,
        "Mach": 0.11,
        "CL_0": 0.3,   "CL_alpha": 5.5,  "CL_q": 4.5,  "CL_deltae": 0.4,  "CL_u": 0.0,
        "CD_0": 0.05,  "CD_alpha": 0.3,  "CD_q": 0.0,  "CD_deltae": 0.0,  "CD_u": 0.0,
        "Cm_0": 0.0,   "Cm_alpha": -0.5, "Cm_q": -3.5, "Cm_deltae": -0.8, "Cm_u": 0.0,
        "CYb": -0.5,"CYp": 0.0,"CYr": 0.0, "CYda": 0.0,"CYdr": 0.1,
        "Clb": -0.05,"Clp": -0.4,"Clr": 0.0,"Clda": 0.2,"Cldr": 0.01,
        "Cnb":  0.05,"Cnp":  0.0,"Cnr": -0.08,"Cnda": 0.0,"Cndr": -0.07,
    },

    "Predator": {
        "name": "Predator",
        "company": "General Atomics", "country": "USA",
        "mass": 1020.0,  "S": 11.45, "c": 0.78, "b": 14.8,
        "Iyy": 5000.0,   "ixx": 6000,"izz": 9000,"ixz": 150,
        "Mach": 0.14,
        "CL_0": 0.25,  "CL_alpha": 5.6,  "CL_q": 4.0,  "CL_deltae": 0.3,  "CL_u": 0.0,
        "CD_0": 0.05,  "CD_alpha": 0.3,  "CD_q": 0.0,  "CD_deltae": 0.0,  "CD_u": 0.0,
        "Cm_0": 0.0,   "Cm_alpha": -0.5, "Cm_q": -4.0, "Cm_deltae": -0.5, "Cm_u": 0.0,
        "CYb": -0.5,"CYp": 0.0,"CYr": 0.0, "CYda": 0.0,"CYdr": 0.1,
        "Clb": -0.05,"Clp": -0.4,"Clr": 0.0,"Clda": 0.2,"Cldr": 0.01,
        "Cnb":  0.05,"Cnp":  0.0,"Cnr": -0.08,"Cnda": 0.0,"Cndr": -0.07,
    },

    "Heron MK1": {
        "name": "Heron MK1",
        "company": "IAI", "country": "Israel",
        "mass": 1150.0,  "S": 12.9,  "c": 0.78, "b": 16.6,
        "Iyy": 4000.0,   "ixx": 4000,"izz": 6000,"ixz": 120,
        "Mach": 0.18,
        "CL_0": 0.4,   "CL_alpha": 5.7,  "CL_q": 5.5,  "CL_deltae": 0.5,  "CL_u": 0.0,
        "CD_0": 0.06,  "CD_alpha": 0.33, "CD_q": 0.0,  "CD_deltae": 0.0,  "CD_u": 0.0,
        "Cm_0": 0.0,   "Cm_alpha": -0.6, "Cm_q": -5.0, "Cm_deltae": -1.0, "Cm_u": 0.0,
        "CYb": -0.5,"CYp": 0.0,"CYr": 0.0, "CYda": 0.0,"CYdr": 0.1,
        "Clb": -0.05,"Clp": -0.4,"Clr": 0.0,"Clda": 0.2,"Cldr": 0.01,
        "Cnb":  0.05,"Cnp":  0.0,"Cnr": -0.08,"Cnda": 0.0,"Cndr": -0.07,
    },

    "Heron MK2": {
        "name": "Heron MK2",
        "company": "IAI", "country": "Israel",
        "mass": 1000.0,  "S": 12.9,  "c": 0.78, "b": 16.6,
        "Iyy": 4000.0,   "ixx": 4000,"izz": 6000,"ixz": 120,
        "Mach": 0.18,
        "CL_0": 0.4,   "CL_alpha": 5.7,  "CL_q": 5.5,  "CL_deltae": 0.5,  "CL_u": 0.0,
        "CD_0": 0.06,  "CD_alpha": 0.33, "CD_q": 0.0,  "CD_deltae": 0.0,  "CD_u": 0.0,
        "Cm_0": 0.0,   "Cm_alpha": -0.6, "Cm_q": -5.0, "Cm_deltae": -1.0, "Cm_u": 0.0,
        "CYb": -0.5,"CYp": 0.0,"CYr": 0.0, "CYda": 0.0,"CYdr": 0.1,
        "Clb": -0.05,"Clp": -0.4,"Clr": 0.0,"Clda": 0.2,"Cldr": 0.01,
        "Cnb":  0.05,"Cnp":  0.0,"Cnr": -0.08,"Cnda": 0.0,"Cndr": -0.07,
    },
}

# Convenience: list of valid aircraft names
AIRCRAFT_NAMES: List[str] = list(_DB.keys())


# ---------------------------------------------------------------------------
# Public accessor
# ---------------------------------------------------------------------------

def get_aircraft_params(name: str) -> Dict[str, Any]:
    """
    Return the complete parameter dict for *name*.

    Also injects derived parameters (U0, rho, q_bar) used by the dynamics engine.

    Raises
    ------
    KeyError if name is not in the database.
    """
    if name not in _DB:
        raise KeyError(
            f"Aircraft '{name}' not found. Available: {AIRCRAFT_NAMES}"
        )
    params = dict(_DB[name])  # shallow copy

    # Inject derived fields required by dynamics and aerodynamics modules
    U0            = params["Mach"] * A_SOUND
    params["U0"]  = U0
    params["rho"] = RHO0
    import math as _m
    params["q_bar"] = 0.5 * RHO0 * U0 * U0

    return params


def list_aircraft() -> List[str]:
    """Return list of all aircraft names in the database."""
    return list(AIRCRAFT_NAMES)


def aircraft_info(name: str) -> str:
    """Return a human-readable summary string for an aircraft."""
    p  = get_aircraft_params(name)
    U0 = p["U0"]
    return (
        f"{name} ({p['company']}, {p['country']}) | "
        f"mass={p['mass']} kg | S={p['S']} m² | b={p['b']} m | "
        f"U0={U0:.1f} m/s ({U0*1.944:.1f} kn)"
    )
