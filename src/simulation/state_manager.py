"""
state_manager.py  –  Simulation state container and history buffer.

AircraftSimState    : dataclass holding the full 12-D integration state
                      plus derived quantities (alpha, beta, airspeed, altitude)
StateHistory        : pre-allocated NumPy arrays for efficient history recording
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class AircraftSimState:
    """
    Full simulation state (12-D NED) with derived quantities.

    Follows the same layout as the nonlinear_model ODE:
      [u, v, w, p, q, r, phi, theta, psi, x_N, x_E, x_D]
    """
    # Body-frame velocities (m/s)
    u: float = 30.0
    v: float = 0.0
    w: float = 0.0

    # Body-frame angular rates (rad/s)
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    # Euler angles (rad)
    phi:   float = 0.0
    theta: float = 0.0
    psi:   float = 0.0

    # NED position (m)
    x_north: float = 0.0
    x_east:  float = 0.0
    x_down:  float = -100.0   # negative = above ground in NED

    # Derived (computed externally, not integrated)
    alpha:    float = 0.0     # angle of attack (rad)
    beta:     float = 0.0     # sideslip angle  (rad)
    airspeed: float = 30.0    # m/s
    altitude: float = 100.0   # m above ground (positive up)

    # ------------------------------------------------------------------

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "AircraftSimState":
        """Create from a 12-D state vector."""
        s = cls()
        s.u, s.v, s.w       = float(arr[0]), float(arr[1]), float(arr[2])
        s.p, s.q, s.r       = float(arr[3]), float(arr[4]), float(arr[5])
        s.phi, s.theta, s.psi = float(arr[6]), float(arr[7]), float(arr[8])
        s.x_north, s.x_east, s.x_down = float(arr[9]), float(arr[10]), float(arr[11])

        # Derived
        airspeed = max(np.sqrt(s.u**2 + s.v**2 + s.w**2), 1e-3)
        s.airspeed = airspeed
        s.altitude = -s.x_down      # NED down → alt
        s.alpha    = float(np.arctan2(s.w, s.u))
        s.beta     = float(np.arcsin(np.clip(s.v / airspeed, -1.0, 1.0)))
        return s

    def to_array(self) -> np.ndarray:
        """Export as 12-D state vector."""
        return np.array([
            self.u, self.v, self.w,
            self.p, self.q, self.r,
            self.phi, self.theta, self.psi,
            self.x_north, self.x_east, self.x_down,
        ])

    @property
    def pos_ned(self) -> np.ndarray:
        return np.array([self.x_north, self.x_east, self.x_down])

    @property
    def vel_body(self) -> np.ndarray:
        return np.array([self.u, self.v, self.w])

    @property
    def omega(self) -> np.ndarray:
        return np.array([self.p, self.q, self.r])

    @property
    def euler(self) -> np.ndarray:
        return np.array([self.phi, self.theta, self.psi])


class StateHistory:
    """
    Pre-allocated history buffer for efficient recording during simulation.

    Parameters
    ----------
    n_steps : expected number of time steps
    """

    STATE_KEYS = [
        "t",
        "u", "v", "w",
        "p", "q", "r",
        "phi", "theta", "psi",
        "x_north", "x_east", "x_down",
        "alpha", "beta", "airspeed", "altitude",
        # control surfaces
        "elevator", "aileron", "rudder", "throttle",
        # desired position
        "des_north", "des_east", "des_down",
    ]

    def __init__(self, n_steps: int):
        self.n_steps   = n_steps
        self._idx      = 0
        self._data: Dict[str, np.ndarray] = {
            k: np.zeros(n_steps) for k in self.STATE_KEYS
        }

    def record(
        self,
        t:     float,
        state: AircraftSimState,
        elevator:  float = 0.0,
        aileron:   float = 0.0,
        rudder:    float = 0.0,
        throttle:  float = 0.0,
        des_pos:   Optional[np.ndarray] = None,
    ) -> None:
        """Record one time step."""
        if self._idx >= self.n_steps:
            return   # buffer full; ignore

        i = self._idx
        d = self._data

        d["t"][i]        = t
        d["u"][i]        = state.u
        d["v"][i]        = state.v
        d["w"][i]        = state.w
        d["p"][i]        = state.p
        d["q"][i]        = state.q
        d["r"][i]        = state.r
        d["phi"][i]      = state.phi
        d["theta"][i]    = state.theta
        d["psi"][i]      = state.psi
        d["x_north"][i]  = state.x_north
        d["x_east"][i]   = state.x_east
        d["x_down"][i]   = state.x_down
        d["alpha"][i]    = state.alpha
        d["beta"][i]     = state.beta
        d["airspeed"][i] = state.airspeed
        d["altitude"][i] = state.altitude
        d["elevator"][i] = elevator
        d["aileron"][i]  = aileron
        d["rudder"][i]   = rudder
        d["throttle"][i] = throttle

        if des_pos is not None:
            d["des_north"][i] = float(des_pos[0])
            d["des_east"][i]  = float(des_pos[1])
            d["des_down"][i]  = float(des_pos[2])

        self._idx += 1

    def trim(self) -> None:
        """Remove unused tail of pre-allocated arrays."""
        n = self._idx
        self._data = {k: v[:n] for k, v in self._data.items()}
        self.n_steps = n

    def get(self, key: str) -> np.ndarray:
        return self._data[key][:self._idx]

    def to_dict(self) -> Dict[str, np.ndarray]:
        return {k: v[:self._idx].copy() for k, v in self._data.items()}

    def to_csv(self, path: str) -> None:
        """Export history to CSV file."""
        import csv, os
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        keys = self.STATE_KEYS
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(keys)
            n = self._idx
            for i in range(n):
                writer.writerow([self._data[k][i] for k in keys])
