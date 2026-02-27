"""
rate_controller.py  –  Angular rate controller and Stability Augmentation System (SAS).

Layer 4 of the ArduPilot control hierarchy.

Three independent rate PIDs:
  PTCH_RATE_P/I/D  →  elevator increment
  ROLL_RATE_P/I    →  aileron  increment
  YAW_RATE_P/I     →  rudder   increment

The SAS is always active and provides damping of the natural modes
(short period and phugoid).  It operates as the innermost feedback loop.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

from control.pid_controller   import PIDController
from control.ardupilot_compat import ArdupilotParams


@dataclass
class RateOutput:
    """Surface deflection increments from the rate / SAS loop (normalised −1..1)."""
    elevator: float = 0.0
    aileron:  float = 0.0
    rudder:   float = 0.0


class RateController:
    """
    Inner-loop angular rate PID controller (SAS).

    Inputs : desired angular rates (p_cmd, q_cmd, r_cmd) from AttitudeController
             + measured angular rates (p, q, r)
    Outputs: normalised surface deflection increments (elevator, aileron, rudder)

    ArduPilot parameters used:
      PTCH_RATE_P, PTCH_RATE_I, PTCH_RATE_D, PTCH_RATE_FF
      ROLL_RATE_P, ROLL_RATE_I, ROLL_RATE_FF
      YAW_RATE_P, YAW_RATE_I
    """

    def __init__(self, ap_params: ArdupilotParams, dt: float = 0.01):
        self.ap = ap_params
        self.dt = dt
        self._build_controllers()

    def _build_controllers(self) -> None:
        ap = self.ap
        self.pitch_rate_pid = PIDController(
            kp=ap.PTCH_RATE_P, ki=ap.PTCH_RATE_I, kd=ap.PTCH_RATE_D,
            output_min=-1.0, output_max=1.0, dt=self.dt,
        )
        self.roll_rate_pid = PIDController(
            kp=ap.ROLL_RATE_P, ki=ap.ROLL_RATE_I, kd=0.0,
            output_min=-1.0, output_max=1.0, dt=self.dt,
        )
        self.yaw_rate_pid = PIDController(
            kp=ap.YAW_RATE_P, ki=ap.YAW_RATE_I, kd=0.0,
            output_min=-1.0, output_max=1.0, dt=self.dt,
        )

    # ------------------------------------------------------------------

    def update(
        self,
        p: float, q: float, r: float,              # measured rates (rad/s)
        p_cmd: float, q_cmd: float, r_cmd: float,  # desired rates  (rad/s)
        dt: float = None,
    ) -> RateOutput:
        """
        Compute surface deflection increments.

        Parameters
        ----------
        p, q, r       : measured body angular rates (rad/s)
        p_cmd, q_cmd, r_cmd : desired angular rates from attitude controller
        dt            : time step (s)

        Returns
        -------
        RateOutput (elevator / aileron / rudder normalised increments)
        """
        if dt is None:
            dt = self.dt

        # Feed-forward (ArduPilot: FF term added before saturation)
        elev  = self.pitch_rate_pid.update(q_cmd - q, dt=dt,
                                            feed_forward=self.ap.PTCH_RATE_FF * q_cmd)
        ail   = self.roll_rate_pid.update (p_cmd - p, dt=dt,
                                            feed_forward=self.ap.ROLL_RATE_FF * p_cmd)
        rud   = self.yaw_rate_pid.update  (r_cmd - r, dt=dt)

        return RateOutput(elevator=elev, aileron=ail, rudder=rud)

    def reload_gains(self, ap_params: ArdupilotParams) -> None:
        """Hot-reload gains."""
        self.ap = ap_params
        self._build_controllers()

    def reset(self) -> None:
        self.pitch_rate_pid.reset()
        self.roll_rate_pid.reset()
        self.yaw_rate_pid.reset()
