"""
pid_controller.py  –  Generic PID controller with anti-windup.

Design mirrors ArduPilot's AC_PID implementation:
  - Proportional, integral, derivative terms
  - Clamping-based anti-windup (integral accumulates only when unsaturated)
  - Optional first-order derivative low-pass filter
  - reset() for mode transitions
"""

from __future__ import annotations

import numpy as np
from utils.math_utils import saturate


class PIDController:
    """
    Discrete-time PID controller with clamping anti-windup.

    Parameters
    ----------
    kp, ki, kd   : P/I/D gains
    output_min   : lower saturation limit
    output_max   : upper saturation limit
    d_lpf_hz     : derivative low-pass filter cutoff (Hz); 0 = no filter
    dt           : default time step (s); can be overridden in update()
    """

    def __init__(
        self,
        kp:         float = 1.0,
        ki:         float = 0.0,
        kd:         float = 0.0,
        output_min: float = -1.0,
        output_max: float =  1.0,
        d_lpf_hz:   float = 20.0,
        dt:         float = 0.01,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.d_lpf_hz   = d_lpf_hz
        self.dt_default = dt

        self._integral:  float = 0.0
        self._prev_error: float = 0.0
        self._d_filtered: float = 0.0
        self._saturated:  bool  = False

    # ------------------------------------------------------------------

    def update(self, error: float, dt: float = None, feed_forward: float = 0.0) -> float:
        """
        Compute PID output for *error* at this timestep.

        Parameters
        ----------
        error        : set-point minus measured value
        dt           : time step (s); uses default if None
        feed_forward : optional feed-forward term added to output

        Returns
        -------
        output : float  (clamped to [output_min, output_max])
        """
        if dt is None or dt <= 0.0:
            dt = self.dt_default

        # --- Derivative (with optional LPF) ---------------------------------
        d_raw = (error - self._prev_error) / dt
        if self.d_lpf_hz > 0.0:
            alpha = 1.0 / (1.0 + 1.0 / (2.0 * np.pi * self.d_lpf_hz * dt))
            self._d_filtered = alpha * self._d_filtered + (1.0 - alpha) * d_raw
        else:
            self._d_filtered = d_raw

        # --- Proportional ----------------------------------------------------
        p_out = self.kp * error

        # --- Integral (clamping anti-windup) ---------------------------------
        # Only integrate when not saturated (ArduPilot clamping method)
        if not self._saturated:
            self._integral += self.ki * error * dt
            # Clamp integral itself to output range
            self._integral = saturate(self._integral, self.output_min, self.output_max)

        # --- Total output ----------------------------------------------------
        output_raw = p_out + self._integral + self.kd * self._d_filtered + feed_forward

        # --- Saturation & anti-windup flag -----------------------------------
        output = saturate(output_raw, self.output_min, self.output_max)
        self._saturated = (output_raw != output)

        self._prev_error = error
        return output

    def reset(self, zero_integrator: bool = True) -> None:
        """Reset controller state (call on mode transitions)."""
        self._prev_error  = 0.0
        self._d_filtered  = 0.0
        self._saturated   = False
        if zero_integrator:
            self._integral = 0.0

    def set_gains(self, kp: float = None, ki: float = None, kd: float = None) -> None:
        """Update gains at runtime."""
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd

    def __repr__(self) -> str:
        return (f"PIDController(kp={self.kp}, ki={self.ki}, kd={self.kd}, "
                f"out=[{self.output_min},{self.output_max}])")
