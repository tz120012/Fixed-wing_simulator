"""
navigation_controller.py  –  Navigation and path-tracking controller.

Implements an L1 navigation law (as used in ArduPilot Plane) to compute
lateral / altitude / airspeed commands from the current position and a
target path segment.

Reference:
  S. Park, J. Deyst, J.P. How, "A New Nonlinear Guidance Logic for
  Trajectory Tracking", AIAA 2004-4900.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

from control.flight_mode_manager import AircraftState, ControlTarget
from utils.math_utils import wrap_angle, saturate


@dataclass
class PathSegment:
    """A straight-line path segment from *start* to *end* in NED (m)."""
    start: np.ndarray   # (3,) NED
    end:   np.ndarray   # (3,) NED
    target_speed: float = 30.0   # m/s

    @property
    def direction(self) -> np.ndarray:
        delta = self.end - self.start
        n = np.linalg.norm(delta)
        if n < 1e-3:
            return np.array([1.0, 0.0, 0.0])
        return delta / n

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end - self.start))


class NavigationController:
    """
    L1 navigation controller for fixed-wing path tracking.

    Parameters
    ----------
    l1_period  : L1 guidance period T (s)  – ArduPilot NAVL1_PERIOD
    l1_damping : L1 damping ratio ζ        – ArduPilot NAVL1_DAMPING
    max_roll   : maximum commanded roll (rad)
    """

    def __init__(
        self,
        l1_period:   float = 25.0,
        l1_damping:  float = 0.75,
        max_roll:    float = np.radians(45.0),
        max_pitch:   float = np.radians(20.0),
        min_pitch:   float = np.radians(-5.0),
        cruise_speed: float = 30.0,
        cruise_alt:  float = 100.0,
    ):
        self.l1_period   = l1_period
        self.l1_damping  = l1_damping
        self.max_roll    = max_roll
        self.max_pitch   = max_pitch
        self.min_pitch   = min_pitch
        self.cruise_speed = cruise_speed
        self.cruise_alt  = cruise_alt

        # Altitude / speed control parameters
        self.k_alt    = 0.05   # altitude error → pitch command (rad/m)
        self.k_speed  = 0.05   # speed error → throttle P gain (1/(m/s))
        self.ki_speed = 0.005  # speed error → throttle I gain (1/(m/s²))
        self._thr_integral = 0.0   # throttle integrator state

    # ------------------------------------------------------------------

    def update(
        self,
        state:   AircraftState,
        segment: PathSegment,
        dt:      float = 0.1,
    ) -> ControlTarget:
        """
        Compute ControlTarget from current state and path segment.

        Parameters
        ----------
        state   : current aircraft state
        segment : desired path segment (NED start/end, m)
        dt      : time step (s)

        Returns
        -------
        ControlTarget with roll_cmd (rad), pitch_cmd (rad), throttle_cmd
        """
        target = ControlTarget()

        # ---- L1 lateral navigation law ------------------------------------
        roll_cmd = self._l1_roll(state, segment)
        target.roll_cmd = saturate(roll_cmd, -self.max_roll, self.max_roll)

        # ---- Desired heading (along path direction) -----------------------
        seg_dir = segment.direction[:2]  # North-East component
        target.yaw_cmd = np.arctan2(seg_dir[1], seg_dir[0])

        # ---- Altitude control → pitch command ----------------------------
        alt_err = state.altitude - self.cruise_alt
        pitch_cmd = -self.k_alt * alt_err   # negative error → climb (positive pitch)
        target.pitch_cmd = saturate(pitch_cmd, self.min_pitch, self.max_pitch)

        # ---- Airspeed / throttle (PI 控制) -----------------------------------
        target.airspeed_cmd = segment.target_speed
        speed_err = segment.target_speed - state.airspeed
        # 积分项（带饱和防积分饱和）
        self._thr_integral += self.ki_speed * speed_err * dt
        self._thr_integral  = saturate(self._thr_integral, -0.4, 0.4)
        thr = saturate(0.5 + self.k_speed * speed_err + self._thr_integral, 0.0, 1.0)
        target.throttle_cmd = thr

        target.altitude_cmd = self.cruise_alt

        return target

    # ------------------------------------------------------------------

    def _l1_roll(self, state: AircraftState, segment: PathSegment) -> float:
        """
        L1 guidance lateral acceleration → roll command.

        Returns desired roll angle in radians.
        """
        # Ground-speed vector (approximate with airspeed and heading)
        V = max(state.airspeed, 5.0)
        track_ang = np.arctan2(state.v + V * np.sin(state.psi),
                               state.u + V * np.cos(state.psi))
        vel_ned = V * np.array([np.cos(state.psi), np.sin(state.psi)])

        # Aircraft position (NE only for lateral law)
        pos_ne = np.array([state.pos_north, state.pos_east])

        # Closest point on segment (NE)
        seg_ne_start = segment.start[:2]
        seg_ne_end   = segment.end[:2]
        seg_vec      = seg_ne_end - seg_ne_start
        seg_len      = max(np.linalg.norm(seg_vec), 1.0)
        seg_dir_ne   = seg_vec / seg_len

        # Cross-track error (signed)
        dp = pos_ne - seg_ne_start
        xtrack = dp[0] * seg_dir_ne[1] - dp[1] * seg_dir_ne[0]  # right = positive

        # L1 distance
        L1 = max(V * self.l1_period / (2 * np.pi), 5.0)

        # Angle between velocity and desired track
        desired_track = np.arctan2(seg_dir_ne[1], seg_dir_ne[0])
        curr_track    = np.arctan2(vel_ned[1], vel_ned[0])
        eta  = wrap_angle(desired_track - curr_track)

        # L1 lateral acceleration
        a_lat = 2.0 * V**2 / L1 * np.sin(eta) - 2.0 * V * xtrack / (L1**2)

        # Convert lateral acceleration to bank angle
        g = 9.80665
        roll_cmd = np.arctan2(a_lat, g)
        return roll_cmd
