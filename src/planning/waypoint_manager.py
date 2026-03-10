"""
waypoint_manager.py  –  Waypoint management and trajectory factory.

Manages a list of NED waypoints, creates trajectories on demand, and
provides the active path segment at any time t.
"""

from __future__ import annotations

import os
import yaml
import numpy as np
from typing import List, Optional, Tuple

from planning.trajectory_base import AbstractTrajectory, TrajectoryState
from planning.minimum_snap    import MinimumSnapTrajectory
from planning.minimum_jerk    import MinimumJerkTrajectory


class WaypointManager:
    """
    Manages mission waypoints and builds trajectory objects.

    Waypoints are stored in NED (m):  [north, east, down].
    Altitudes given as 'positive-up' are converted internally to NED down.

    Parameters
    ----------
    average_speed : cruise speed for segment-time estimation (m/s)
    traj_type     : 'minimum_snap' | 'minimum_jerk'
    yaw_mode      : 'yaw_follow' | 'zero' | 'fixed'
    loop          : if True, trajectory loops back to first waypoint
    """

    def __init__(
        self,
        average_speed: float = 30.0,
        traj_type:     str   = "minimum_snap",
        yaw_mode:      str   = "yaw_follow",
        loop:          bool  = False,
    ):
        self.average_speed = average_speed
        self.traj_type     = traj_type
        self.yaw_mode      = yaw_mode
        self.loop          = loop

        self._waypoints_ned: List[np.ndarray] = []
        self._trajectory: Optional[AbstractTrajectory] = None

    # ------------------------------------------------------------------
    # Waypoint management
    # ------------------------------------------------------------------

    def add_waypoint(
        self,
        north: float,
        east:  float,
        alt_m: float,   # positive UP; converted to NED down internally
    ) -> None:
        """Add a single waypoint (in NED format after conversion)."""
        self._waypoints_ned.append(np.array([north, east, -alt_m]))
        self._trajectory = None   # invalidate cached trajectory

    def add_waypoints_ned(self, wps: np.ndarray) -> None:
        """
        Add multiple waypoints already in NED format.

        Parameters
        ----------
        wps : (n, 3) array  [north, east, down] in metres
        """
        for wp in wps:
            self._waypoints_ned.append(np.asarray(wp, dtype=float))
        self._trajectory = None

    def clear_waypoints(self) -> None:
        self._waypoints_ned = []
        self._trajectory    = None

    def load_from_yaml(self, path: str) -> None:
        """
        Load waypoints from a trajectory YAML config file.

        Expected format::

            type: minimum_snap
            average_speed: 30.0
            yaw_mode: yaw_follow
            loop: false
            waypoints:
              - [north_m, east_m, alt_m]   # alt = positive up
        """
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Trajectory config not found: {path}")
        with open(path, "r") as f:
            cfg = yaml.safe_load(f) or {}

        self.traj_type     = cfg.get("type",          self.traj_type)
        self.average_speed = cfg.get("average_speed", self.average_speed)
        self.yaw_mode      = cfg.get("yaw_mode",      self.yaw_mode)
        self.loop          = bool(cfg.get("loop", self.loop))

        self.clear_waypoints()
        for wp in cfg.get("waypoints", []):
            n, e, a = float(wp[0]), float(wp[1]), float(wp[2])
            self._waypoints_ned.append(np.array([n, e, -a]))  # alt → down

    def save_to_yaml(self, path: str) -> None:
        """Save current waypoint list to a YAML file."""
        wps = [[float(w[0]), float(w[1]), float(-w[2])]
               for w in self._waypoints_ned]
        cfg = {
            "type":          self.traj_type,
            "average_speed": float(self.average_speed),
            "yaw_mode":      self.yaw_mode,
            "loop":          self.loop,
            "waypoints":     wps,
        }
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            yaml.dump(cfg, f, default_flow_style=False)

    # ------------------------------------------------------------------
    # Trajectory generation
    # ------------------------------------------------------------------

    def build_trajectory(self) -> AbstractTrajectory:
        """
        Build and cache the trajectory object from current waypoints.

        Returns
        -------
        AbstractTrajectory instance ready to be queried with desired_state(t).

        Raises
        ------
        ValueError if fewer than 2 waypoints are defined.
        """
        if len(self._waypoints_ned) < 2:
            raise ValueError("At least 2 waypoints required to build a trajectory.")

        wps = np.array(self._waypoints_ned)

        if self.loop and not np.allclose(wps[0], wps[-1], atol=1.0):
            wps = np.vstack([wps, wps[:1]])   # close the loop

        kwargs = dict(
            waypoints=wps,
            average_speed=self.average_speed,
            yaw_mode=self.yaw_mode,
        )

        if self.traj_type == "minimum_snap":
            self._trajectory = MinimumSnapTrajectory(**kwargs)
        elif self.traj_type == "minimum_jerk":
            self._trajectory = MinimumJerkTrajectory(**kwargs)
        else:
            raise ValueError(f"Unknown trajectory type: {self.traj_type}")

        return self._trajectory

    @property
    def trajectory(self) -> AbstractTrajectory:
        """Return the cached trajectory, building it if necessary."""
        if self._trajectory is None:
            self.build_trajectory()
        return self._trajectory

    @property
    def total_duration(self) -> float:
        """Total trajectory duration in seconds."""
        return float(getattr(self.trajectory, "T_total", 0.0))

    # ------------------------------------------------------------------
    # Active segment access
    # ------------------------------------------------------------------

    def get_active_segment(
        self, t: float
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Return (wp_start, wp_end, T_remaining) for current time *t*.

        Returns
        -------
        wp_start, wp_end : NED positions (m)
        T_remaining      : time remaining in current segment (s)
        """
        traj = self.trajectory
        T_cum = traj.T_cumulative
        t_c   = np.clip(t, 0.0, traj.T_total)
        seg   = int(np.clip(
            np.searchsorted(T_cum[1:], t_c, side='right'),
            0, len(self._waypoints_ned) - 2,
        ))
        t_remaining = T_cum[seg + 1] - t_c
        return (
            self._waypoints_ned[seg],
            self._waypoints_ned[min(seg + 1, len(self._waypoints_ned) - 1)],
            float(t_remaining),
        )

    # ------------------------------------------------------------------

    def desired_state(self, t: float) -> TrajectoryState:
        """Convenience: delegate to underlying trajectory."""
        return self.trajectory.desired_state(t)
