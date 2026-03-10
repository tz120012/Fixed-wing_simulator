"""dynamics package – physics and kinematics of fixed-wing aircraft."""

from dynamics.linear_model        import LinearModel, LinearAnalysisResult
from dynamics.nonlinear_model     import NonlinearModel, Controls, TrimResult, NonlinearSimResult
from dynamics.aerodynamics        import compute_aero_forces, AeroForces
from dynamics.coordinate_transform import (
    dcm_from_euler,
    body_to_ned,
    ned_to_body,
    euler_rates,
    wind_to_body_frame,
    airspeed_vector,
)

__all__ = [
    "LinearModel", "LinearAnalysisResult",
    "NonlinearModel", "Controls", "TrimResult", "NonlinearSimResult",
    "compute_aero_forces", "AeroForces",
    "dcm_from_euler", "body_to_ned", "ned_to_body",
    "euler_rates", "wind_to_body_frame", "airspeed_vector",
]
