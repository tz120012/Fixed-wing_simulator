"""
navigation_controller.py  –  Navigation and path-tracking controller.

Implements:
  - L1 lateral navigation law (ArduPilot NAVL1_PERIOD / NAVL1_DAMPING)
  - TECS (Total Energy Control System) for altitude & airspeed control

References:
  S. Park, J. Deyst, J.P. How, "A New Nonlinear Guidance Logic for
  Trajectory Tracking", AIAA 2004-4900.
  AP_TECS (ArduPilot), Paul Riseborough 2013.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import Optional

from control.flight_mode_manager import AircraftState, ControlTarget
from control.tecs_controller import TECSController
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
    L1 navigation controller + TECS for fixed-wing path tracking.

    Parameters
    ----------
    l1_period    : L1 guidance period T (s)  – ArduPilot NAVL1_PERIOD
    l1_damping   : L1 damping ratio ζ        – ArduPilot NAVL1_DAMPING
    max_roll     : maximum commanded roll (rad)
    tecs_kwargs  : keyword arguments forwarded to TECSController.__init__
    """

    def __init__(
        self,
        l1_period:    float = 25.0,
        l1_damping:   float = 0.75,
        max_roll:     float = np.radians(45.0),
        cruise_speed: float = 40.0,
        cruise_alt:   float = 100.0,
        # --- TECS 参数 ---
        tecs_max_climb_rate: float = 5.0,    # TECS_CLMB_MAX  (m/s)
        tecs_min_sink_rate:  float = 2.0,    # TECS_SINK_MIN  (m/s)
        tecs_max_sink_rate:  float = 5.0,    # TECS_SINK_MAX  (m/s)
        tecs_time_const:     float = 5.0,    # TECS_TIME_CONST (s)
        tecs_thr_damp:       float = 0.5,    # TECS_THR_DAMP
        tecs_ptch_damp:      float = 0.3,    # TECS_PTCH_DAMP
        tecs_integ_gain:     float = 0.3,    # TECS_INTEG_GAIN
        tecs_spd_weight:     float = 1.0,    # TECS_SPDWEIGHT (0=高度优先 2=速度优先)
        tecs_roll_comp:      float = 10.0,   # TECS_RLL2THR
        tecs_pitch_min:      float = None,   # rad (None → -15°)
        tecs_pitch_max:      float = None,   # rad (None → +15°)
        tecs_thr_cruise:     float = 0.5,    # 巡航油门（前馈基准）
        tecs_thr_min:        float = 0.0,
        tecs_thr_max:        float = 1.0,
        airspeed_min:        float = 28.0,
        airspeed_max:        float = 60.0,
        tecs_hdem_tconst:    float = 1.5,    # TECS_HDEM_TCONST 高度需求低通时间常数
    ):
        self.l1_period   = l1_period
        self.l1_damping  = l1_damping
        self.max_roll    = max_roll
        self.cruise_speed = cruise_speed
        self.cruise_alt  = cruise_alt

        # 高度目标低通滤波状态（防止轨迹超调导致 TECS 接收异常高度指令）
        self._alt_dem_lpf  = cruise_alt   # 初始化为巡航高度
        self._alt_dem_tconst = tecs_hdem_tconst  # 与 TECS 同等时间常数

        # 构建 TECS
        self.tecs = TECSController(
            max_climb_rate  = tecs_max_climb_rate,
            min_sink_rate   = tecs_min_sink_rate,
            max_sink_rate   = tecs_max_sink_rate,
            time_const      = tecs_time_const,
            thr_damp        = tecs_thr_damp,
            ptch_damp       = tecs_ptch_damp,
            integ_gain      = tecs_integ_gain,
            spd_weight      = tecs_spd_weight,
            roll_comp       = tecs_roll_comp,
            pitch_min       = (tecs_pitch_min if tecs_pitch_min is not None
                               else np.radians(-15.0)),
            pitch_max       = (tecs_pitch_max if tecs_pitch_max is not None
                               else np.radians(15.0)),
            thr_cruise      = tecs_thr_cruise,
            thr_min         = tecs_thr_min,
            thr_max         = tecs_thr_max,
            airspeed_min    = airspeed_min,
            airspeed_max    = airspeed_max,
            airspeed_cruise = cruise_speed,
            hgt_dem_tconst  = tecs_hdem_tconst,
        )
        self.tecs.reset()

    # ------------------------------------------------------------------

    def reset(self, state: AircraftState = None):
        """重置 TECS 积分器（模式切换或重新启动时调用）。"""
        if state is not None:
            self._alt_dem_lpf = state.altitude   # 同步高度低通初始值
            self.tecs.reset(
                height   = state.altitude,
                airspeed = state.airspeed,
                pitch    = state.theta,
            )
        else:
            self.tecs.reset()

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

        # ---- 目标高度（从路段终点提取）------------------------------------
        seg_alt = -segment.end[2] if len(segment.end) > 2 else self.cruise_alt
        raw_alt = seg_alt if seg_alt > 0 else self.cruise_alt
        # Pass the clamped altitude directly to TECS.
        # Internal rate-limiting and smoothing are handled inside TECS
        # (_update_height_demand), so an extra outer filter is not needed.
        target_alt = raw_alt
        target.altitude_cmd = target_alt

        # ---- 估计爬升率（使用 NED 惯性系垂直速度）--------------------------
        # Vertical velocity (positive up) = u·sin(θ) − w·cos(θ)
        # This equals V·sin(γ) where γ = θ − α is the flight-path angle.
        # Using θ alone would over-estimate climb rate by V·sin(α_trim).
        V = max(state.airspeed, 3.0)
        climb_rate_est = (state.u * np.sin(state.theta)
                          - state.w * np.cos(state.theta))   # m/s, positive = up

        # ---- 估计体轴加速度（前向）----------------------------------------
        # 仿真中通常可从状态微分获取；这里用空速变化估算
        # 若 state 中没有 ax，用 0（TECS 仍可正常工作）
        accel_body_x = getattr(state, 'ax', 0.0)

        # ---- TECS 更新 ----------------------------------------------------
        tecs_out = self.tecs.update(
            height       = state.altitude,
            climb_rate   = climb_rate_est,
            airspeed     = state.airspeed,
            accel_body_x = accel_body_x,
            roll_rad     = state.phi,
            hgt_dem      = target_alt,
            airspeed_dem = segment.target_speed,
            dt           = dt,
        )

        # ---- 将 TECS 输出写入 ControlTarget -----------------------------
        target.pitch_cmd    = tecs_out.pitch_dem
        target.throttle_cmd = tecs_out.throttle_dem
        target.airspeed_cmd = segment.target_speed

        return target

    # ------------------------------------------------------------------

    def _l1_roll(self, state: AircraftState, segment: PathSegment) -> float:
        """
        Standard ArduPilot L1 look-ahead point guidance law.

        Algorithm (ref: AP_L1_Control.cpp):
          1. Project aircraft onto segment, find along-track distance.
          2. Place look-ahead point at (along_track + L1) clamped to segment end.
          3. Compute angle eta between current ground-track velocity and look-ahead
             direction (use body velocity components u,v to get true ground track).
          4. a_lat = 2V²/L1 * sin(eta)
          5. roll_cmd = atan(a_lat / g)

        Uses u/v body components to derive the NE ground-track angle, which is
        more accurate than heading psi when sideslip or wind is present.

        Returns desired roll angle in radians.
        """
        V  = max(state.airspeed, 5.0)
        g  = 9.80665

        # L1 look-ahead distance
        L1 = max(V * self.l1_period / (2.0 * np.pi), 5.0)

        # Aircraft NE position
        pos_ne = np.array([state.pos_north, state.pos_east])

        # Segment geometry (NE plane)
        seg_start = segment.start[:2]
        seg_end   = segment.end[:2]
        seg_vec   = seg_end - seg_start
        seg_len   = np.linalg.norm(seg_vec)

        if seg_len < 1.0:
            # Degenerate segment – hold current heading
            return 0.0

        seg_dir = seg_vec / seg_len

        # Along-track distance of aircraft projection onto segment
        dp          = pos_ne - seg_start
        along_track = np.dot(dp, seg_dir)

        # --- Choose look-ahead point -------------------------------------------
        # If the aircraft has already passed the segment end (along_track > seg_len),
        # point directly at the segment end rather than continuing along the old
        # heading.  This prevents the common "nose-high overshoot" where the
        # aircraft keeps flying in the original direction far past the waypoint.
        if along_track >= seg_len:
            # Past the end: steer toward segment endpoint
            to_end     = seg_end - pos_ne
            to_end_len = np.linalg.norm(to_end)
            if to_end_len < 0.5:
                return 0.0
            desired_track = np.arctan2(to_end[1], to_end[0])
        else:
            # Normal case: look-ahead point on segment
            look_ahead_dist = float(np.clip(along_track + L1, 0.0, seg_len))
            wp_l1 = seg_start + look_ahead_dist * seg_dir

            to_l1     = wp_l1 - pos_ne
            to_l1_len = np.linalg.norm(to_l1)

            if to_l1_len < 0.5:
                return 0.0

            desired_track = np.arctan2(to_l1[1], to_l1[0])

        # Current ground track angle derived from body velocity projected to NE.
        # Rotating body [u, v] to NED North-East frame:
        #   V_north = u*cos(psi) - v*sin(psi)
        #   V_east  = u*sin(psi) + v*cos(psi)
        # This correctly accounts for sideslip, unlike using psi directly.
        cos_psi = np.cos(state.psi)
        sin_psi = np.sin(state.psi)
        v_north = state.u * cos_psi - state.v * sin_psi
        v_east  = state.u * sin_psi + state.v * cos_psi
        curr_track = np.arctan2(v_east, v_north)

        # eta: signed angle from current track to desired track
        eta = wrap_angle(desired_track - curr_track)

        # Lateral acceleration command
        a_lat = 2.0 * V**2 / L1 * np.sin(eta)

        # Convert to bank angle
        roll_cmd = np.arctan2(a_lat, g)
        return roll_cmd
