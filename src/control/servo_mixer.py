"""
servo_mixer.py  –  Actuator allocation and output limiting.

Layer 5 (innermost) of the ArduPilot control hierarchy.

Combines the surface deflection increments from the rate controller with
the throttle command from the navigation / mode layer, applies:
  - Amplitude limits (LIM_PITCH_MAX/MIN, LIM_ROLL_CD → degrees → rad)
  - Rate limits (deg/s)
  - Coordinated turn rudder compensation
  - Final normalised output [-1, 1]  (or [0, 1] for throttle)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

from control.ardupilot_compat import ArdupilotParams
from utils.math_utils          import saturate


@dataclass
class ServoOutput:
    """Final normalised surface and throttle commands (sent to dynamics)."""
    elevator: float = 0.0   # [-1, 1]  positive = nose up
    aileron:  float = 0.0   # [-1, 1]  positive = right wing down
    rudder:   float = 0.0   # [-1, 1]  positive = nose right
    throttle: float = 0.0   # [ 0, 1]

    def to_radians(
        self,
        elev_max_rad: float = np.radians(25.0),
        ail_max_rad:  float = np.radians(20.0),
        rud_max_rad:  float = np.radians(25.0),
    ):
        """
        Convert normalised outputs to surface deflections in radians.

        Returns
        -------
        (de, da, dr) in radians
        """
        de = self.elevator * elev_max_rad
        da = self.aileron  * ail_max_rad
        dr = self.rudder   * rud_max_rad
        return de, da, dr


class ServoMixer:
    """
    Servo mixer: maps control increments → final servo commands.

    Parameters
    ----------
    ap_params   : ArdupilotParams
    rate_limit  : maximum surface deflection rate (deg/s)
    dt          : default time step (s)
    """

    def __init__(
        self,
        ap_params:  ArdupilotParams,
        rate_limit: float = 60.0,   # deg/s
        dt:         float = 0.01,
    ):
        self.ap         = ap_params
        self.rate_limit = rate_limit   # deg/s
        self.dt         = dt

        # Previous output for rate limiting
        self._prev = ServoOutput()

    # ------------------------------------------------------------------

    def update(
        self,
        elev_in:  float,   # normalised increment from rate controller
        ail_in:   float,
        rud_in:   float,
        throttle: float,   # 0–1 from navigation / mode layer
        phi:      float,   # actual roll  (rad) – for coordinated turn
        p:        float,   # actual roll rate (rad/s)
        dt:       float = None,
    ) -> ServoOutput:
        """
        Compute final servo commands.

        Parameters
        ----------
        elev_in, ail_in, rud_in : normalised increments (−1..1)
        throttle  : 0–1 throttle command
        phi       : actual roll angle (rad) – for coordinated turn
        p         : roll rate (rad/s)
        dt        : time step (s)

        Returns
        -------
        ServoOutput with elevator / aileron / rudder / throttle
        """
        if dt is None:
            dt = self.dt
        ap = self.ap

        # --- Elevator limits from ArduPilot params ----------------------
        # LIM_PITCH_MAX/MIN are in degrees; normalise to [-1, 1]
        # Elevator surface travel assumed ±25 deg
        ELEV_TRAVEL_DEG = 25.0
        elev_max = np.radians(ap.LIM_PITCH_MAX) / np.radians(ELEV_TRAVEL_DEG)
        elev_min = np.radians(ap.LIM_PITCH_MIN) / np.radians(ELEV_TRAVEL_DEG)
        elevator = saturate(elev_in, elev_min, elev_max)

        # --- Aileron limits from LIM_ROLL_CD ----------------------------
        # Aileron surface travel assumed ±20 deg; use roll limit as guide
        AIL_TRAVEL_DEG = 20.0
        roll_limit_rad = np.radians(ap.LIM_ROLL_DEG)
        # Approximation: full aileron when roll error > 2×roll_limit
        ail_lim = min(1.0, roll_limit_rad / np.radians(AIL_TRAVEL_DEG) * 1.5)
        aileron = saturate(ail_in, -ail_lim, ail_lim)

        # --- Coordinated turn rudder compensation -----------------------
        # Add rudder to cancel adverse yaw: rud_coord ∝ roll_rate
        coord_gain = 0.05   # empirical, tunable
        rud_coord  = coord_gain * p
        rudder = saturate(rud_in + rud_coord, -1.0, 1.0)

        # --- Throttle ---------------------------------------------------
        throttle = saturate(throttle, ap.THR_MIN, ap.THR_MAX)

        # --- Rate limiting (deg/s → normalised/s) -----------------------
        max_delta = (self.rate_limit / 100.0) * dt   # very approximate
        elevator = saturate(elevator,
                            self._prev.elevator - max_delta,
                            self._prev.elevator + max_delta)
        aileron  = saturate(aileron,
                            self._prev.aileron  - max_delta,
                            self._prev.aileron  + max_delta)
        rudder   = saturate(rudder,
                            self._prev.rudder   - max_delta,
                            self._prev.rudder   + max_delta)

        out = ServoOutput(elevator=elevator, aileron=aileron,
                          rudder=rudder, throttle=throttle)
        self._prev = out
        return out

    def reset(self) -> None:
        self._prev = ServoOutput()
