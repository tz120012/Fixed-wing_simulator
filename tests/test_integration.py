"""
test_integration.py  –  Integration tests for the full simulation pipeline.

These tests run a complete simulation loop (not just unit functions) to verify
that all modules cooperate correctly and that the system remains numerically
stable.

Test scenarios:
  1. TB2, open-loop (trim hold), 5 s → state should not diverge
  2. TB2, closed-loop (STABILIZE mode, no trajectory), 5 s → state stable
  3. TB2, closed-loop (AUTO mode, 2-waypoint trajectory), 10 s → state stable
  4. Anka, open-loop, 5 s → state stable  (checks database + dynamics pipeline)
  5. run_linear_analysis on TB2 → backward-compat result
  6. step() API → produces valid AircraftSimState per step
"""

import os
import sys
import math

import numpy as np
import pytest

# ──────────────────────────────────────────────────────────────
# Ensure src/ is importable
# ──────────────────────────────────────────────────────────────
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC  = os.path.join(_ROOT, "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from simulation.simulator  import FixedWingSimulator, SimulationResult
from simulation.state_manager import AircraftSimState
from dynamics.linear_model import LinearAnalysisResult


# ──────────────────────────────────────────────────────────────
# Helper
# ──────────────────────────────────────────────────────────────

def _check_not_diverged(history_dict: dict, max_altitude_m: float = 10_000.0,
                         max_speed_m_s: float = 500.0) -> None:
    """Assert that key state variables did not blow up (numerical divergence)."""
    alt   = history_dict["altitude"]
    spd   = history_dict["airspeed"]
    theta = np.degrees(history_dict["theta"])

    assert np.all(np.isfinite(alt)),   "Altitude contains non-finite values"
    assert np.all(np.isfinite(spd)),   "Airspeed contains non-finite values"
    assert np.max(np.abs(alt))  < max_altitude_m, \
        f"Altitude diverged: max={np.max(np.abs(alt)):.0f} m"
    assert np.max(spd) < max_speed_m_s, \
        f"Airspeed diverged: max={np.max(spd):.1f} m/s"
    assert np.min(spd) >= 0.0, \
        f"Airspeed went negative: min={np.min(spd):.2f}"
    assert np.max(np.abs(theta)) < 90.0, \
        f"Pitch diverged: max |theta|={np.max(np.abs(theta)):.1f} deg"


# ──────────────────────────────────────────────────────────────
# Config dir (use the package config/)
# ──────────────────────────────────────────────────────────────
CONFIG_DIR = os.path.join(_ROOT, "config")


# ═══════════════════════════════════════════════════════════════
# 1. Open-loop trim-hold (no control)
# ═══════════════════════════════════════════════════════════════

class TestOpenLoopTrimHold:

    def test_tb2_open_loop_5s(self):
        """TB2 open-loop (trim hold) for 5 s: state must not diverge."""
        sim = FixedWingSimulator(
            aircraft_name="TB2",
            config_dir=CONFIG_DIR,
            dt=0.02,
            duration=5.0,
            initial_mode="STABILIZE",
            wind_type="NONE",
        )
        result = sim.run(closed_loop=False)
        assert isinstance(result, SimulationResult)
        h = result.history.to_dict()
        _check_not_diverged(h)

    def test_result_summary_str(self):
        """SimulationResult.summary() should return a non-empty string."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.05, duration=2.0,
        )
        result = sim.run(closed_loop=False)
        s = result.summary()
        assert isinstance(s, str) and len(s) > 0

    def test_anka_open_loop_5s(self):
        """Anka open-loop for 5 s."""
        sim = FixedWingSimulator(
            aircraft_name="Anka", config_dir=CONFIG_DIR,
            dt=0.02, duration=5.0, wind_type="NONE",
        )
        result = sim.run(closed_loop=False)
        h = result.history.to_dict()
        _check_not_diverged(h)


# ═══════════════════════════════════════════════════════════════
# 2. Closed-loop, STABILIZE mode (no trajectory)
# ═══════════════════════════════════════════════════════════════

class TestClosedLoopStabilize:

    def test_tb2_stabilize_5s(self):
        """TB2 in STABILIZE mode for 5 s should remain stable."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=5.0,
            initial_mode="STABILIZE", wind_type="NONE",
        )
        result = sim.run(closed_loop=True)
        h = result.history.to_dict()
        _check_not_diverged(h)

    def test_with_fixed_wind(self):
        """TB2 with constant headwind should still be stable."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=5.0,
            initial_mode="STABILIZE", wind_type="FIXED",
        )
        result = sim.run(closed_loop=True)
        h = result.history.to_dict()
        _check_not_diverged(h)

    def test_history_arrays_correct_length(self):
        """History arrays should have length ≈ duration/dt."""
        dt = 0.05
        duration = 3.0
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=dt, duration=duration, initial_mode="STABILIZE",
        )
        result = sim.run(closed_loop=False)
        h = result.history.to_dict()
        expected = int(duration / dt) + 1
        assert len(h["t"]) >= expected - 2   # allow ±2 for rounding

    def test_history_time_monotone(self):
        """Time vector must be strictly increasing."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=3.0,
        )
        result = sim.run(closed_loop=False)
        t = result.history.to_dict()["t"]
        assert np.all(np.diff(t) > 0), "Time vector is not strictly increasing"


# ═══════════════════════════════════════════════════════════════
# 3. Closed-loop, AUTO mode + trajectory
# ═══════════════════════════════════════════════════════════════

class TestClosedLoopAuto:

    def test_tb2_auto_with_trajectory_10s(self):
        """
        TB2 in AUTO mode with a 2-waypoint trajectory for 10 s.
        Trajectory is added programmatically (not from YAML).
        """
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=10.0,
            initial_mode="AUTO", wind_type="NONE",
        )
        # Add a 2-waypoint trajectory (300 m straight north at 50 m alt)
        sim.wp_mgr.add_waypoint(0.0, 0.0, 50.0)
        sim.wp_mgr.add_waypoint(300.0, 0.0, 50.0)

        result = sim.run(closed_loop=True)
        h = result.history.to_dict()
        _check_not_diverged(h)

    def test_tb2_auto_moves_north(self):
        """
        In AUTO mode heading north, x_north should increase over time.
        """
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=10.0,
            initial_mode="AUTO", wind_type="NONE",
        )
        sim.wp_mgr.add_waypoint(0.0,   0.0,  50.0)
        sim.wp_mgr.add_waypoint(500.0, 0.0,  50.0)

        result = sim.run(closed_loop=True)
        h = result.history.to_dict()
        x_north = h["x_north"]
        # Aircraft should have moved at least 10 m north in 10 s
        assert x_north[-1] > x_north[0] + 10.0, \
            f"Aircraft did not move north: Δx_north = {x_north[-1]-x_north[0]:.1f} m"

    def test_minimum_jerk_trajectory(self):
        """AUTO mode with minimum-jerk trajectory should also be stable."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=8.0,
            initial_mode="AUTO", wind_type="NONE",
            traj_type="minimum_jerk",
        )
        sim.wp_mgr.add_waypoint(0.0, 0.0, 50.0)
        sim.wp_mgr.add_waypoint(200.0, 100.0, 50.0)
        sim.wp_mgr.traj_type = "minimum_jerk"

        result = sim.run(closed_loop=True)
        h = result.history.to_dict()
        _check_not_diverged(h)


# ═══════════════════════════════════════════════════════════════
# 4. Backward-compatible linear analysis
# ═══════════════════════════════════════════════════════════════

class TestLinearAnalysis:

    def test_run_linear_analysis_returns_result(self):
        """run_linear_analysis() should return a LinearAnalysisResult."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.01, duration=20.0,
        )
        result = sim.run_linear_analysis(duration=10.0)
        assert isinstance(result, LinearAnalysisResult)

    def test_linear_analysis_modes(self):
        """Linear analysis should return 2 longitudinal modes for TB2."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
        )
        result = sim.run_linear_analysis(duration=10.0)
        assert len(result.modes) == 2

    def test_linear_analysis_summary_str(self):
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
        )
        result = sim.run_linear_analysis(duration=5.0)
        s = result.summary()
        assert "TB2" in s or "Mode" in s or len(s) > 10

    def test_all_aircraft_linear_analysis(self):
        """All 7 aircraft should complete linear analysis without error."""
        from models.aircraft_database import AIRCRAFT_NAMES
        for name in AIRCRAFT_NAMES:
            sim = FixedWingSimulator(aircraft_name=name, config_dir=CONFIG_DIR,
                                     dt=0.01, duration=5.0)
            result = sim.run_linear_analysis(duration=5.0)
            assert result is not None, f"Linear analysis returned None for {name}"
            assert len(result.modes) == 2, \
                f"Expected 2 modes for {name}, got {len(result.modes)}"


# ═══════════════════════════════════════════════════════════════
# 5. Step-by-step API (for Reflex UI integration)
# ═══════════════════════════════════════════════════════════════

class TestStepAPI:

    def test_init_step_returns_state(self):
        """init_step() should return a valid AircraftSimState."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=5.0,
        )
        state = sim.init_step()
        assert isinstance(state, AircraftSimState)
        assert state.airspeed > 0.0

    def test_step_advances_time(self):
        """Repeated step() calls should return valid states without error."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=10.0, initial_mode="STABILIZE",
        )
        state0 = sim.init_step()
        alt0 = state0.altitude
        for _ in range(10):
            state = sim.step(0.02)
        # After 10 steps the simulation should have advanced (integrator internal time)
        # Just verify the returned state is valid
        assert isinstance(state, AircraftSimState)
        assert math.isfinite(state.airspeed)

    def test_step_state_finite(self):
        """State returned by step() must always have finite values."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.02, duration=5.0, initial_mode="STABILIZE",
        )
        sim.init_step()
        for _ in range(50):
            state = sim.step(0.02)
            assert math.isfinite(state.airspeed), \
                "Airspeed became non-finite during step()"
            assert math.isfinite(state.altitude), \
                "Altitude became non-finite during step()"
            assert math.isfinite(state.phi), \
                "Roll became non-finite during step()"

    def test_step_api_consistent_with_run(self):
        """
        5 s of step() calls should produce similar final state to run().
        (Approximate match due to control update timing differences.)
        """
        duration = 5.0
        dt       = 0.02

        # run() path
        sim_run = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=dt, duration=duration, initial_mode="STABILIZE", wind_type="NONE",
        )
        res_run = sim_run.run(closed_loop=False)
        final_alt_run = res_run.history.to_dict()["altitude"][-1]

        # step() path
        sim_step = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=dt, duration=duration, initial_mode="STABILIZE", wind_type="NONE",
        )
        sim_step.init_step()
        n = int(duration / dt)
        state_step = None
        for _ in range(n):
            state_step = sim_step.step(dt)

        final_alt_step = state_step.altitude

        # Both should be in roughly the same altitude range (within 200 m)
        assert abs(final_alt_step - final_alt_run) < 200.0, \
            (f"step() and run() produced very different altitudes: "
             f"step={final_alt_step:.1f} m, run={final_alt_run:.1f} m")


# ═══════════════════════════════════════════════════════════════
# 6. StateHistory and AircraftSimState
# ═══════════════════════════════════════════════════════════════

class TestStateHistory:

    def test_from_array_to_array_roundtrip(self):
        """from_array → to_array should reproduce the input vector."""
        arr = np.array([30.0, 0.1, 0.5,
                         0.01, 0.02, 0.0,
                         0.05, 0.1, 1.57,
                         100.0, 50.0, -200.0])
        state = AircraftSimState.from_array(arr)
        arr2  = state.to_array()
        np.testing.assert_allclose(arr2, arr, atol=1e-12)

    def test_from_array_derived_quantities(self):
        """Derived quantities (alpha, airspeed, altitude) should be correct."""
        u, w = 30.0, 1.0
        arr = np.zeros(12)
        arr[0] = u; arr[2] = w
        arr[11] = -500.0   # NED down = −500 → alt = 500 m
        state = AircraftSimState.from_array(arr)

        expected_airspeed = math.sqrt(u**2 + w**2)
        expected_alpha    = math.atan2(w, u)
        expected_alt      = 500.0

        assert abs(state.airspeed - expected_airspeed) < 1e-10
        assert abs(state.alpha    - expected_alpha)    < 1e-10
        assert abs(state.altitude - expected_alt)      < 1e-10

    def test_history_to_dict_keys(self):
        """to_dict() should contain all expected keys."""
        sim = FixedWingSimulator(
            aircraft_name="TB2", config_dir=CONFIG_DIR,
            dt=0.05, duration=1.0,
        )
        result = sim.run(closed_loop=False)
        h = result.history.to_dict()
        required_keys = ["t", "u", "v", "w", "p", "q", "r",
                          "phi", "theta", "psi",
                          "x_north", "x_east", "x_down",
                          "airspeed", "altitude", "alpha"]
        for key in required_keys:
            assert key in h, f"Key '{key}' missing from history dict"
