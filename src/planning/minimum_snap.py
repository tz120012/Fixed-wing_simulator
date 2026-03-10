"""
minimum_snap.py  –  Minimum-Snap piecewise polynomial trajectory.

Each spatial axis (N, E, D, and yaw) is represented by a degree-(2*M−1)
polynomial on each segment, where M = 4 for minimum snap (4th derivative).

The implementation directly follows the matrix-construction approach from
Quadcopter_SimCon/Simulation/trajectory.py (minSomethingTraj) adapted for
fixed-wing (3D + yaw, with optional stop or through-point constraints).

Reference:
  D. Mellinger & V. Kumar, "Minimum Snap Trajectory Generation and
  Control for Quadrotors", ICRA 2011.
"""

from __future__ import annotations

import numpy as np
from typing import List, Optional, Tuple

from planning.trajectory_base import AbstractTrajectory, TrajectoryState


def _get_poly_cc(n: int, k: int, t: float) -> np.ndarray:
    """
    Coefficient vector for the k-th derivative of an n-th order polynomial
    evaluated at time *t*.

    Returns (n,) array such that  p^(k)(t) = coeff @ cc.

    Parameters
    ----------
    n : polynomial order (number of coefficients)
    k : derivative order
    t : evaluation time
    """
    cc = np.zeros(n)
    for i in range(k, n):
        # Factorial coefficient
        fac = 1
        for j in range(k):
            fac *= (i - j)
        cc[i] = fac * (t ** (i - k))
    return cc


def minimum_snap_coeffs(
    waypoints: np.ndarray,
    T_segments: np.ndarray,
    deriv_order: int = 4,
    stop_at_waypoints: bool = False,
) -> np.ndarray:
    """
    Compute minimum-snap (deriv_order=4) polynomial coefficients.

    Parameters
    ----------
    waypoints       : (n_wp, d) array of waypoints (d spatial dimensions)
    T_segments      : (n_wp-1,) array of segment durations (s)
    deriv_order     : 4 = minimum snap, 3 = minimum jerk, 2 = min accel
    stop_at_waypoints : if True, velocity = 0 at each intermediate waypoint

    Returns
    -------
    coeffs : (n_segments, 2*deriv_order, d) array of polynomial coefficients
             coeffs[seg, :, dim] are the coefficients for segment *seg*, dimension *dim*
    """
    n_wp   = len(waypoints)
    n_seg  = n_wp - 1
    n_dim  = waypoints.shape[1]
    M      = 2 * deriv_order   # polynomial order per segment

    coeffs_all = np.zeros((n_seg, M, n_dim))

    for dim in range(n_dim):
        # Build linear system A @ x = b  (each row is one constraint)
        n_total = M * n_seg
        A = np.zeros((n_total, n_total))
        b = np.zeros(n_total)

        row = 0
        for seg in range(n_seg):
            T  = T_segments[seg]
            col_start = seg * M

            # Constraint 1: start position
            A[row, col_start:col_start + M] = _get_poly_cc(M, 0, 0.0)
            b[row] = waypoints[seg, dim]
            row += 1

            # Constraint 2: end position
            A[row, col_start:col_start + M] = _get_poly_cc(M, 0, T)
            b[row] = waypoints[seg + 1, dim]
            row += 1

        # Boundary conditions at start: derivatives 1..(deriv_order-1) = 0
        for k in range(1, deriv_order):
            col_start = 0
            A[row, col_start:col_start + M] = _get_poly_cc(M, k, 0.0)
            b[row] = 0.0
            row += 1

        # Boundary conditions at end: derivatives 1..(deriv_order-1) = 0
        for k in range(1, deriv_order):
            col_start = (n_seg - 1) * M
            T_last = T_segments[-1]
            A[row, col_start:col_start + M] = _get_poly_cc(M, k, T_last)
            b[row] = 0.0
            row += 1

        # Continuity at intermediate waypoints
        for seg in range(n_seg - 1):
            T   = T_segments[seg]
            cs  = seg * M
            csn = (seg + 1) * M
            # Derivatives 1 .. (2*deriv_order - 1)
            for k in range(1, M):
                if row >= n_total:
                    break
                A[row, cs:cs   + M] =  _get_poly_cc(M, k, T)
                A[row, csn:csn + M] = -_get_poly_cc(M, k, 0.0)
                b[row] = 0.0
                row += 1

                if stop_at_waypoints and k == 1:
                    # Velocity = 0 at waypoint (overwrite the continuity row)
                    A[row - 1, :]   = 0.0
                    A[row - 1, cs:cs + M] = _get_poly_cc(M, 1, T)
                    b[row - 1] = 0.0

        # Solve
        try:
            if np.linalg.cond(A) > 1e10:
                print("[MinSnap] WARNING: ill-conditioned system (large segment T?)")
            x = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            x = np.linalg.lstsq(A, b, rcond=None)[0]

        for seg in range(n_seg):
            coeffs_all[seg, :, dim] = x[seg * M:(seg + 1) * M]

    return coeffs_all


def _eval_poly(coeffs_seg: np.ndarray, t_local: float, deriv: int = 0) -> np.ndarray:
    """
    Evaluate polynomial (or its k-th derivative) at local time t_local.

    Parameters
    ----------
    coeffs_seg : (M, d) coefficients for one segment
    t_local    : time within the segment (0..T_seg)
    deriv      : derivative order (0=position, 1=velocity, 2=accel)

    Returns
    -------
    (d,) value
    """
    M, d = coeffs_seg.shape
    result = np.zeros(d)
    for dim in range(d):
        cc = _get_poly_cc(M, deriv, t_local)
        result[dim] = cc @ coeffs_seg[:, dim]
    return result


class MinimumSnapTrajectory(AbstractTrajectory):
    """
    Minimum-Snap trajectory through a list of NED waypoints.

    Parameters
    ----------
    waypoints      : (n, 3) NED waypoints (m)
    T_segments     : (n-1,) segment durations (s); if None, computed from average_speed
    average_speed  : m/s, used when T_segments is None
    yaw_mode       : 'zero' | 'yaw_follow' | 'fixed'
    yaw_waypoints  : (n,) desired yaw at each waypoint (rad); used when yaw_mode='fixed'
    stop_at_waypoints : zero velocity at each intermediate waypoint
    """

    def __init__(
        self,
        waypoints:     np.ndarray,
        T_segments:    Optional[np.ndarray] = None,
        average_speed: float = 30.0,
        yaw_mode:      str   = "yaw_follow",
        yaw_waypoints: Optional[np.ndarray] = None,
        stop_at_waypoints: bool = False,
    ):
        waypoints = np.asarray(waypoints, dtype=float)
        assert waypoints.ndim == 2 and waypoints.shape[1] == 3, \
            "waypoints must be (n, 3)"
        assert len(waypoints) >= 2, "Need at least 2 waypoints"

        self.waypoints   = waypoints
        self.yaw_mode    = yaw_mode
        self.yaw_wps     = yaw_waypoints

        # --- Segment times --------------------------------------------------
        if T_segments is None:
            dists = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
            T_segments = np.maximum(dists / max(average_speed, 0.1), 0.5)
        self.T_segments   = np.asarray(T_segments, dtype=float)
        self.T_cumulative = np.concatenate([[0.0], np.cumsum(self.T_segments)])
        self.T_total      = float(self.T_cumulative[-1])

        # --- Compute polynomial coefficients --------------------------------
        self.coeffs = minimum_snap_coeffs(
            waypoints, self.T_segments,
            deriv_order=4, stop_at_waypoints=stop_at_waypoints,
        )

        # Optional yaw trajectory
        if yaw_mode == "yaw_follow":
            self._yaw_from_vel = True
        else:
            self._yaw_from_vel = False
            if yaw_waypoints is not None:
                self.yaw_coeffs = minimum_snap_coeffs(
                    np.column_stack([yaw_waypoints, np.zeros_like(yaw_waypoints),
                                     np.zeros_like(yaw_waypoints)]),
                    self.T_segments, deriv_order=2,
                )

    # ------------------------------------------------------------------

    def desired_state(self, t: float) -> TrajectoryState:
        t = float(t)
        t_clamped = np.clip(t, 0.0, self.T_total)

        # Find segment index
        seg = np.searchsorted(self.T_cumulative[1:], t_clamped, side='right')
        seg = int(np.clip(seg, 0, len(self.T_segments) - 1))
        t_local = t_clamped - self.T_cumulative[seg]

        pos = _eval_poly(self.coeffs[seg], t_local, deriv=0)
        vel = _eval_poly(self.coeffs[seg], t_local, deriv=1)
        acc = _eval_poly(self.coeffs[seg], t_local, deriv=2)

        # Yaw
        if self._yaw_from_vel and np.linalg.norm(vel[:2]) > 0.5:
            yaw = np.arctan2(vel[1], vel[0])  # NE plane: E/N
        else:
            yaw = 0.0

        yaw_rate = 0.0

        return TrajectoryState(pos=pos, vel=vel, acc=acc,
                               yaw=yaw, yaw_rate=yaw_rate)

    def reset(self) -> None:
        pass   # stateless
