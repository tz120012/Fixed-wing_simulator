"""
attitude_controller.py  –  Three-axis attitude (angle) controller.

Implements three independent PID controllers for roll, pitch, and yaw
using ArduPilot Plane parameter naming.  The attitude controller takes
desired Euler angles and produces desired angular rate commands (or direct
surface deflection increments) that are fed to the rate_controller.

ArduPilot reference parameters used:
  PTCH_P, PTCH_D
  ROLL_P, ROLL_D
  YAW_P
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

from control.pid_controller   import PIDController
from control.ardupilot_compat import ArdupilotParams
from utils.math_utils          import wrap_angle, saturate


@dataclass
class AttitudeOutput:
    """Desired angular rate commands out of the attitude controller (rad/s)."""
    roll_rate_cmd:  float = 0.0
    pitch_rate_cmd: float = 0.0
    yaw_rate_cmd:   float = 0.0


class AttitudeController:
    """
    Attitude controller: desired Euler angles → desired angular rates.

    Mirrors ArduPilot Plane's stabilise_roll / stabilise_pitch / stabilise_yaw.

    Parameters
    ----------
    ap_params : ArdupilotParams  – holds PTCH_P/D, ROLL_P/D, YAW_P
    dt        : default time step (s)
    """

    # Output limits for desired angular rates (rad/s)
    MAX_ROLL_RATE  = np.radians(120.0)   # 120 deg/s
    MAX_PITCH_RATE = np.radians(60.0)    # 60 deg/s
    MAX_YAW_RATE   = np.radians(45.0)    # 45 deg/s

    def __init__(self, ap_params: ArdupilotParams, dt: float = 0.01):
        self.ap = ap_params
        self.dt = dt
        self._build_controllers()

    def _build_controllers(self) -> None:
        ap = self.ap
        # Roll: uses ROLL_P as P gain, ROLL_D as derivative (ArduPilot convention)
        self.roll_pid = PIDController(
            kp=ap.ROLL_P, ki=0.0, kd=ap.ROLL_D,
            output_min=-self.MAX_ROLL_RATE,
            output_max= self.MAX_ROLL_RATE,
            dt=self.dt,
        )
        # Pitch: PTCH_P, PTCH_D
        self.pitch_pid = PIDController(
            kp=ap.PTCH_P, ki=0.0, kd=ap.PTCH_D,
            output_min=-self.MAX_PITCH_RATE,
            output_max= self.MAX_PITCH_RATE,
            dt=self.dt,
        )
        # Yaw / heading: YAW_P only (simple proportional)
        self.yaw_pid = PIDController(
            kp=ap.YAW_P, ki=0.0, kd=0.0,
            output_min=-self.MAX_YAW_RATE,
            output_max= self.MAX_YAW_RATE,
            dt=self.dt,
        )

    # ------------------------------------------------------------------

    def update(
        self,
        phi:       float,   # actual roll  (rad)
        theta:     float,   # actual pitch (rad)
        psi:       float,   # actual yaw   (rad)
        roll_cmd:  float,   # desired roll  (rad)
        pitch_cmd: float,   # desired pitch (rad)
        yaw_cmd:   float,   # desired yaw   (rad)
        dt:        float = None,
    ) -> AttitudeOutput:
        """
        Compute desired angular rate commands.

        Parameters
        ----------
        phi, theta, psi     : actual Euler angles (rad)
        roll_cmd, pitch_cmd, yaw_cmd : desired Euler angles (rad)
        dt                  : time step (s)

        Returns
        -------
        AttitudeOutput with roll/pitch/yaw rate commands (rad/s)
        """
        if dt is None:
            dt = self.dt

        # Roll error (wrap to ±π)
        roll_err  = wrap_angle(roll_cmd  - phi)
        # Pitch error
        pitch_err = wrap_angle(pitch_cmd - theta)
        # Yaw / heading error (wrap to ±π)
        yaw_err   = wrap_angle(yaw_cmd   - psi)

        p_cmd = self.roll_pid.update(roll_err,   dt=dt)
        q_cmd = self.pitch_pid.update(pitch_err, dt=dt)
        r_cmd = self.yaw_pid.update(yaw_err,     dt=dt)

        return AttitudeOutput(
            roll_rate_cmd  = p_cmd,
            pitch_rate_cmd = q_cmd,
            yaw_rate_cmd   = r_cmd,
        )

    def reload_gains(self, ap_params: ArdupilotParams) -> None:
        """Hot-reload gains from updated ArdupilotParams."""
        self.ap = ap_params
        self._build_controllers()

    def reset(self) -> None:
        """Reset all controllers (call on mode transitions)."""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
