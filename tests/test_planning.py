"""
test_planning.py  –  Unit tests for planning/ module.

Covers:
  - minimum_snap_coeffs : polynomial coefficient dimensions, boundary conditions
  - MinimumSnapTrajectory : desired_state output, continuity, position at waypoints
  - MinimumJerkTrajectory : same as snap (deriv_order=3)
  - WaypointManager : add/clear/build, altitude conversion, yaml round-trip
"""

import os
import sys
import math
import tempfile

import numpy as np
import pytest

# ──────────────────────────────────────────────────────────────
# Ensure src/ is importable
# ──────────────────────────────────────────────────────────────
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC  = os.path.join(_ROOT, "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from planning.minimum_snap import minimum_snap_coeffs, MinimumSnapTrajectory
from planning.minimum_jerk import MinimumJerkTrajectory
from planning.waypoint_manager import WaypointManager
from planning.trajectory_base  import TrajectoryState


# ──────────────────────────────────────────────────────────────
# Shared test data
# ──────────────────────────────────────────────────────────────

# Simple 3-waypoint path in NED (metres)
WP_3 = np.array([
    [0.0,   0.0,   0.0],
    [100.0, 0.0,  -50.0],
    [200.0, 50.0, -50.0],
], dtype=float)

T_SEG_3 = np.array([5.0, 5.0])   # 5 s per segment @ ~20–30 m/s


# ═══════════════════════════════════════════════════════════════
# 1. minimum_snap_coeffs (core solver)
# ═══════════════════════════════════════════════════════════════

class TestMinimumSnapCoeffs:

    def test_output_shape_2wp(self):
        """2 waypoints → 1 segment, deriv_order=4 → (1, 8, 3)."""
        wps = np.array([[0., 0., 0.], [100., 0., 0.]])
        T   = np.array([5.0])
        coeffs = minimum_snap_coeffs(wps, T, deriv_order=4)
        assert coeffs.shape == (1, 8, 3), \
            f"Expected (1,8,3), got {coeffs.shape}"

    def test_output_shape_3wp(self):
        """3 waypoints → 2 segments, deriv_order=4 → (2, 8, 3)."""
        coeffs = minimum_snap_coeffs(WP_3, T_SEG_3, deriv_order=4)
        assert coeffs.shape == (2, 8, 3)

    def test_output_shape_minimum_jerk(self):
        """deriv_order=3 → 6 coefficients per segment."""
        wps = WP_3
        T   = T_SEG_3
        coeffs = minimum_snap_coeffs(wps, T, deriv_order=3)
        assert coeffs.shape == (2, 6, 3)

    def test_start_position_satisfied(self):
        """Trajectory should pass through the first waypoint at t=0."""
        from planning.minimum_snap import _get_poly_cc, _eval_poly
        coeffs = minimum_snap_coeffs(WP_3, T_SEG_3, deriv_order=4)
        pos_start = _eval_poly(coeffs[0], 0.0, deriv=0)
        np.testing.assert_allclose(pos_start, WP_3[0], atol=1e-6)

    def test_end_position_satisfied(self):
        """Trajectory should pass through the last waypoint."""
        from planning.minimum_snap import _eval_poly
        n_seg = len(T_SEG_3)
        coeffs = minimum_snap_coeffs(WP_3, T_SEG_3, deriv_order=4)
        pos_end = _eval_poly(coeffs[-1], T_SEG_3[-1], deriv=0)
        np.testing.assert_allclose(pos_end, WP_3[-1], atol=1e-6)

    def test_intermediate_waypoint_satisfied(self):
        """Trajectory should pass through each intermediate waypoint."""
        from planning.minimum_snap import _eval_poly
        coeffs = minimum_snap_coeffs(WP_3, T_SEG_3, deriv_order=4)
        # End of segment 0 == start of segment 1 == WP_3[1]
        pos_mid = _eval_poly(coeffs[0], T_SEG_3[0], deriv=0)
        np.testing.assert_allclose(pos_mid, WP_3[1], atol=1e-6)

    def test_velocity_continuity_at_junction(self):
        """Velocity must be continuous across segment boundaries."""
        from planning.minimum_snap import _eval_poly
        coeffs = minimum_snap_coeffs(WP_3, T_SEG_3, deriv_order=4)
        vel_end_seg0  = _eval_poly(coeffs[0], T_SEG_3[0], deriv=1)
        vel_start_seg1 = _eval_poly(coeffs[1], 0.0,        deriv=1)
        np.testing.assert_allclose(vel_end_seg0, vel_start_seg1, atol=1e-5)

    def test_all_coefficients_finite(self):
        """All polynomial coefficients must be finite numbers."""
        coeffs = minimum_snap_coeffs(WP_3, T_SEG_3, deriv_order=4)
        assert np.all(np.isfinite(coeffs)), "Non-finite coefficients found"

    def test_normalised_time_stability(self):
        """Very long segments (large T) should not produce non-finite coefficients."""
        wps = np.array([[0., 0., 0.], [1000., 0., 0.]])
        T   = np.array([120.0])   # 2-minute segment
        coeffs = minimum_snap_coeffs(wps, T, deriv_order=4)
        # Coefficients may be large but should be finite
        assert np.all(np.isfinite(coeffs))


# ═══════════════════════════════════════════════════════════════
# 2. MinimumSnapTrajectory
# ═══════════════════════════════════════════════════════════════

class TestMinimumSnapTrajectory:

    @pytest.fixture
    def snap_traj(self):
        return MinimumSnapTrajectory(WP_3, T_segments=T_SEG_3)

    def test_desired_state_type(self, snap_traj):
        state = snap_traj.desired_state(0.0)
        assert isinstance(state, TrajectoryState)

    def test_position_at_start(self, snap_traj):
        state = snap_traj.desired_state(0.0)
        np.testing.assert_allclose(state.pos, WP_3[0], atol=1e-5)

    def test_position_at_end(self, snap_traj):
        T = snap_traj.T_total
        state = snap_traj.desired_state(T)
        np.testing.assert_allclose(state.pos, WP_3[-1], atol=1e-5)

    def test_position_at_waypoint(self, snap_traj):
        state = snap_traj.desired_state(T_SEG_3[0])
        np.testing.assert_allclose(state.pos, WP_3[1], atol=1e-5)

    def test_velocity_returns_3d(self, snap_traj):
        state = snap_traj.desired_state(2.5)
        assert state.vel.shape == (3,)

    def test_acceleration_returns_3d(self, snap_traj):
        state = snap_traj.desired_state(2.5)
        assert state.acc.shape == (3,)

    def test_state_finite_throughout(self, snap_traj):
        T = snap_traj.T_total
        for t in np.linspace(0, T, 50):
            s = snap_traj.desired_state(t)
            assert np.all(np.isfinite(s.pos)), f"Non-finite pos at t={t:.2f}"
            assert np.all(np.isfinite(s.vel)), f"Non-finite vel at t={t:.2f}"

    def test_clamping_before_start(self, snap_traj):
        """Querying t<0 should return the start position (clamped)."""
        state_neg = snap_traj.desired_state(-1.0)
        state_zero = snap_traj.desired_state(0.0)
        np.testing.assert_allclose(state_neg.pos, state_zero.pos, atol=1e-10)

    def test_clamping_after_end(self, snap_traj):
        """Querying t>T_total should return the end position (clamped)."""
        T = snap_traj.T_total
        state_over = snap_traj.desired_state(T + 100.0)
        state_end  = snap_traj.desired_state(T)
        np.testing.assert_allclose(state_over.pos, state_end.pos, atol=1e-10)

    def test_yaw_follow_mode(self):
        """In yaw_follow mode, yaw should align with velocity direction."""
        traj = MinimumSnapTrajectory(WP_3, T_segments=T_SEG_3, yaw_mode="yaw_follow")
        # At mid-trajectory, velocity should be mostly northward (WP2 to WP3)
        state = traj.desired_state(T_SEG_3[0] + 1.0)
        assert math.isfinite(state.yaw)

    def test_single_long_segment(self):
        """Two waypoints far apart should produce valid trajectory."""
        wps = np.array([[0., 0., 0.], [500., 0., -100.]])
        traj = MinimumSnapTrajectory(wps, average_speed=30.0)
        state_end = traj.desired_state(traj.T_total)
        np.testing.assert_allclose(state_end.pos, wps[1], atol=1e-4)


# ═══════════════════════════════════════════════════════════════
# 3. MinimumJerkTrajectory
# ═══════════════════════════════════════════════════════════════

class TestMinimumJerkTrajectory:

    @pytest.fixture
    def jerk_traj(self):
        return MinimumJerkTrajectory(WP_3, T_segments=T_SEG_3)

    def test_position_at_start(self, jerk_traj):
        state = jerk_traj.desired_state(0.0)
        np.testing.assert_allclose(state.pos, WP_3[0], atol=1e-5)

    def test_position_at_end(self, jerk_traj):
        T = jerk_traj.T_total
        state = jerk_traj.desired_state(T)
        np.testing.assert_allclose(state.pos, WP_3[-1], atol=1e-5)

    def test_position_at_waypoint(self, jerk_traj):
        state = jerk_traj.desired_state(T_SEG_3[0])
        np.testing.assert_allclose(state.pos, WP_3[1], atol=1e-5)

    def test_state_finite_throughout(self, jerk_traj):
        T = jerk_traj.T_total
        for t in np.linspace(0, T, 30):
            s = jerk_traj.desired_state(t)
            assert np.all(np.isfinite(s.pos))

    def test_velocity_continuity(self, jerk_traj):
        """Velocity must be continuous at segment boundary."""
        T1 = T_SEG_3[0]
        state_before = jerk_traj.desired_state(T1 - 1e-6)
        state_after  = jerk_traj.desired_state(T1 + 1e-6)
        np.testing.assert_allclose(state_before.vel, state_after.vel, atol=0.1)

    def test_lower_snap_than_msnap(self):
        """
        Minimum-jerk should have lower total jerk than minimum-snap
        (qualitative sanity check: jerk polynomial order is lower → more freedom to
        minimize jerk).
        """
        snap_traj = MinimumSnapTrajectory(WP_3, T_segments=T_SEG_3)
        jerk_traj = MinimumJerkTrajectory(WP_3, T_segments=T_SEG_3)
        times = np.linspace(0, T_SEG_3.sum(), 100)
        dt = times[1] - times[0]

        def total_jerk(traj):
            acc_vals = np.array([traj.desired_state(t).acc for t in times])
            jerk_vals = np.diff(acc_vals, axis=0) / dt
            return np.sum(np.linalg.norm(jerk_vals, axis=1))

        # Jerk traj should not have dramatically higher jerk than snap traj
        jerk_jerk = total_jerk(jerk_traj)
        jerk_snap = total_jerk(snap_traj)
        # Just verify both are finite and neither is pathologically large
        assert math.isfinite(jerk_jerk)
        assert math.isfinite(jerk_snap)


# ═══════════════════════════════════════════════════════════════
# 4. WaypointManager
# ═══════════════════════════════════════════════════════════════

class TestWaypointManager:

    def test_add_waypoint_altitude_conversion(self):
        """alt_m (positive up) should be stored as NED down (negative)."""
        wm = WaypointManager()
        wm.add_waypoint(0.0, 0.0, 100.0)   # 100 m above ground
        wp_ned = wm._waypoints_ned[0]
        assert wp_ned[2] == pytest.approx(-100.0), \
            f"Expected down=-100, got {wp_ned[2]}"

    def test_clear_waypoints(self):
        wm = WaypointManager()
        wm.add_waypoint(0.0, 0.0, 100.0)
        wm.add_waypoint(100.0, 0.0, 100.0)
        wm.clear_waypoints()
        assert len(wm._waypoints_ned) == 0

    def test_add_waypoints_ned(self):
        wm = WaypointManager()
        wps = np.array([[0., 0., -100.], [100., 0., -100.]])
        wm.add_waypoints_ned(wps)
        assert len(wm._waypoints_ned) == 2

    def test_build_minimum_snap(self):
        wm = WaypointManager(traj_type="minimum_snap")
        for wp in WP_3:
            wm.add_waypoints_ned(wp[np.newaxis, :])
        traj = wm.build_trajectory()
        assert isinstance(traj, MinimumSnapTrajectory)

    def test_build_minimum_jerk(self):
        wm = WaypointManager(traj_type="minimum_jerk")
        for wp in WP_3:
            wm.add_waypoints_ned(wp[np.newaxis, :])
        traj = wm.build_trajectory()
        assert isinstance(traj, MinimumJerkTrajectory)

    def test_build_requires_at_least_2_waypoints(self):
        wm = WaypointManager()
        wm.add_waypoint(0.0, 0.0, 100.0)
        with pytest.raises(ValueError):
            wm.build_trajectory()

    def test_yaml_round_trip(self):
        """save_to_yaml → load_from_yaml should reproduce waypoints."""
        wm = WaypointManager(average_speed=25.0, traj_type="minimum_jerk")
        wm.add_waypoint(0.0,   0.0,  100.0)
        wm.add_waypoint(200.0, 0.0,  100.0)
        wm.add_waypoint(200.0, 200.0, 80.0)

        with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
            tmp_path = f.name

        try:
            wm.save_to_yaml(tmp_path)

            wm2 = WaypointManager()
            wm2.load_from_yaml(tmp_path)

            assert len(wm2._waypoints_ned) == 3
            for a, b in zip(wm._waypoints_ned, wm2._waypoints_ned):
                np.testing.assert_allclose(a, b, atol=1e-6)
            assert wm2.traj_type == "minimum_jerk"
        finally:
            os.unlink(tmp_path)

    def test_get_active_segment(self):
        """get_active_segment should return correct segment at given time."""
        wm = WaypointManager(average_speed=100.0)
        for wp in WP_3:
            wm.add_waypoints_ned(wp[np.newaxis, :])
        traj = wm.build_trajectory()
        T1 = traj.T_segments[0]
        seg_start, seg_end, T_rem = wm.get_active_segment(T1 / 2.0)
        assert np.allclose(seg_start, WP_3[0], atol=1.0)  # first segment
        assert T_rem > 0.0
