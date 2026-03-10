"""simulation package – main simulation engine and support modules."""

from simulation.simulator    import FixedWingSimulator, SimulationResult
from simulation.integrator   import Dopri5Integrator, RK45Integrator
from simulation.state_manager import AircraftSimState, StateHistory

__all__ = [
    "FixedWingSimulator", "SimulationResult",
    "Dopri5Integrator", "RK45Integrator",
    "AircraftSimState", "StateHistory",
]
