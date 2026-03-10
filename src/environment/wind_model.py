"""
wind_model.py  –  Wind field models for fixed-wing simulation.

Port and extension of Quadcopter_SimCon/Simulation/utils/windModel.py
adapted for NED convention and fixed-wing flight envelopes.

Supported types:
  NONE       – zero wind
  FIXED      – constant wind vector
  SINE       – sinusoidal superposition of multiple harmonics
  RANDOMSINE – random mean + sinusoidal fluctuations (turbulence-like)
"""

import numpy as np
from typing import Tuple


class Wind:
    """
    Wind field generator.

    Parameters
    ----------
    wind_type : str – one of NONE | FIXED | SINE | RANDOMSINE
    speed     : float – mean/nominal wind speed (m/s)
    direction_deg : float – wind FROM direction (met convention, deg;
                             0=from North, 90=from East)
    """

    TYPES = ("NONE", "FIXED", "SINE", "RANDOMSINE")

    def __init__(
        self,
        wind_type:      str   = "NONE",
        speed:          float = 5.0,
        direction_deg:  float = 270.0,  # west wind (from west = toward east)
        seed:           int   = 42,
    ):
        wind_type = wind_type.upper()
        if wind_type not in self.TYPES:
            raise ValueError(f"Unknown wind type '{wind_type}'. Choose from {self.TYPES}")

        self.wind_type     = wind_type
        self.speed         = float(speed)
        self.direction_deg = float(direction_deg)

        rng = np.random.default_rng(seed)

        # Pre-compute NED unit vector for fixed component
        # "Wind FROM direction_deg" means wind blows TOWARD (direction_deg + 180)
        heading_rad = np.deg2rad(direction_deg + 180.0)
        self._fixed_ned = self.speed * np.array([
            np.cos(heading_rad),   # North
            np.sin(heading_rad),   # East
            0.0,                   # Down (no vertical mean wind)
        ])

        # --- SINE / RANDOMSINE parameters (3 sinusoids per axis) ---
        if wind_type in ("SINE", "RANDOMSINE"):
            n_sin = 3
            # Frequencies: 0.1–0.5 Hz (slow turbulence-like)
            self._freqs  = rng.uniform(0.1, 0.5, (3, n_sin))  # (axis, harmonic)
            self._phases = rng.uniform(0, 2 * np.pi, (3, n_sin))

            if wind_type == "SINE":
                self._amps = np.full((3, n_sin), self.speed / n_sin)
            else:  # RANDOMSINE
                self._amps  = rng.uniform(0, self.speed, (3, n_sin))
                self._means = rng.uniform(-self.speed * 0.5,
                                          self.speed * 0.5, 3)

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def get_wind_ned(self, t: float) -> np.ndarray:
        """
        Return NED wind vector at time *t*.

        Returns
        -------
        (3,) array  [v_north, v_east, v_down]  in m/s
        """
        if self.wind_type == "NONE":
            return np.zeros(3)

        if self.wind_type == "FIXED":
            return self._fixed_ned.copy()

        if self.wind_type == "SINE":
            w = np.zeros(3)
            for ax in range(3):
                for k in range(self._freqs.shape[1]):
                    w[ax] += self._amps[ax, k] * np.sin(
                        2 * np.pi * self._freqs[ax, k] * t + self._phases[ax, k]
                    )
            return w

        if self.wind_type == "RANDOMSINE":
            w = self._means.copy()
            for ax in range(3):
                for k in range(self._freqs.shape[1]):
                    w[ax] += self._amps[ax, k] * np.sin(
                        2 * np.pi * self._freqs[ax, k] * t + self._phases[ax, k]
                    )
            return w

        return np.zeros(3)

    def __repr__(self) -> str:
        return (f"Wind(type={self.wind_type}, speed={self.speed} m/s, "
                f"dir={self.direction_deg} deg)")
