"""
minimum_jerk.py  –  Minimum-Jerk piecewise polynomial trajectory.

Minimum jerk corresponds to deriv_order=3 (5th-order polynomial per segment).
Reuses the minimum_snap coefficient solver with deriv_order=3.
"""

from __future__ import annotations

import numpy as np
from typing import Optional

from planning.trajectory_base import AbstractTrajectory, TrajectoryState
from planning.minimum_snap   import minimum_snap_coeffs, _eval_poly


class MinimumJerkTrajectory(AbstractTrajectory):
    """
    Minimum-Jerk trajectory (5th-order polynomial per segment).

    Interface identical to MinimumSnapTrajectory.
    """

    def __init__(
        self,
        waypoints:     np.ndarray,
        T_segments:    Optional[np.ndarray] = None,
        average_speed: float = 30.0,
        yaw_mode:      str   = "yaw_follow",
        stop_at_waypoints: bool = False,
    ):
        waypoints = np.asarray(waypoints, dtype=float)
        assert waypoints.ndim == 2 and waypoints.shape[1] == 3

        self.waypoints = waypoints
        self.yaw_mode  = yaw_mode

        if T_segments is None:
            dists = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
            T_segments = np.maximum(dists / max(average_speed, 0.1), 0.5)
        self.T_segments   = np.asarray(T_segments, dtype=float)
        self.T_cumulative = np.concatenate([[0.0], np.cumsum(self.T_segments)])
        self.T_total      = float(self.T_cumulative[-1])

        self.coeffs = minimum_snap_coeffs(
            waypoints, self.T_segments,
            deriv_order=3, stop_at_waypoints=stop_at_waypoints,
        )

    def desired_state(self, t: float) -> TrajectoryState:
        t_clamped = float(np.clip(t, 0.0, self.T_total))

        seg = int(np.clip(
            np.searchsorted(self.T_cumulative[1:], t_clamped, side='right'),
            0, len(self.T_segments) - 1,
        ))
        t_local = t_clamped - self.T_cumulative[seg]

        pos = _eval_poly(self.coeffs[seg], t_local, deriv=0)
        vel = _eval_poly(self.coeffs[seg], t_local, deriv=1)
        acc = _eval_poly(self.coeffs[seg], t_local, deriv=2)

        yaw = 0.0
        if self.yaw_mode == "yaw_follow" and np.linalg.norm(vel[:2]) > 0.5:
            yaw = np.arctan2(vel[1], vel[0])

        return TrajectoryState(pos=pos, vel=vel, acc=acc, yaw=yaw)

    def reset(self) -> None:
        pass
