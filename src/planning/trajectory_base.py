"""
trajectory_base.py  –  Abstract base class for all trajectory types.

Defines the common interface used by the simulator:
  desired_state(t) → TrajectoryState
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


@dataclass
class TrajectoryState:
    """Desired trajectory state at a given time instant."""
    pos:      np.ndarray = field(default_factory=lambda: np.zeros(3))  # NED (m)
    vel:      np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s
    acc:      np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s²
    yaw:      float = 0.0        # desired yaw angle (rad)
    yaw_rate: float = 0.0        # desired yaw rate (rad/s)


class AbstractTrajectory(ABC):
    """Abstract trajectory – all trajectory types must implement desired_state()."""

    @abstractmethod
    def desired_state(self, t: float) -> TrajectoryState:
        """
        Return desired trajectory state at time *t* (seconds).

        Parameters
        ----------
        t : float  – current simulation time (s)

        Returns
        -------
        TrajectoryState
        """
        ...

    def reset(self) -> None:
        """Reset any internal state (e.g. segment index)."""
        pass
