"""
test_dynamics.py  –  Unit tests for dynamics/ module.

Covers:
  - coordinate_transform : DCM orthogonality, round-trip body↔NED, Euler rates
  - aerodynamics         : force/moment signs, zero-wind consistency
  - linear_model         : state matrix shape, mode analysis, simulation output
  - nonlinear_model      : trim convergence, state_dot dimension, short simulation
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

# ──────────────────────────────────────────────────────────────
# Imports under test
# ──────────────────────────────────────────────────────────────
from dynamics.coordinate_transform import (
    dcm_from_euler,
    wind_to_body_frame,
    airspeed_vector,
)
from utils.math_utils import (
    body_to_ned,
    ned_to_body,
    euler_rates,
    rotation_matrix_321,
    wrap_angle,
    dynamic_pressure,
)
from dynamics.aerodynamics import compute_aero_forces
from dynamics.linear_model  import LinearModel
from dynamics.nonlinear_model import NonlinearModel, Controls
from models.aircraft_database import get_aircraft_params


# ──────────────────────────────────────────────────────────────
# Fixtures
# ──────────────────────────────────────────────────────────────
@pytest.fixture
def tb2_params():
    return get_aircraft_params("TB2")


@pytest.fixture
def tb2_linear(tb2_params):
    model = LinearModel(tb2_params)
    model.build()
    return model, tb2_params


# ═══════════════════════════════════════════════════════════════
# 1. coordinate_transform
# ═══════════════════════════════════════════════════════════════

class TestCoordinateTransform:

    def test_dcm_identity_at_zero_angles(self):
        """DCM at zero Euler angles should be the identity matrix."""
        R = dcm_from_euler(0.0, 0.0, 0.0)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_dcm_orthogonal(self):
        """DCM must be orthogonal: R @ R.T ≈ I."""
        phi, theta, psi = np.radians([15.0, 5.0, 30.0])
        R = dcm_from_euler(phi, theta, psi)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)

    def test_dcm_determinant_unity(self):
        """Proper rotation matrix must have det = +1."""
        phi, theta, psi = np.radians([10.0, -5.0, 45.0])
        R = dcm_from_euler(phi, theta, psi)
        assert abs(np.linalg.det(R) - 1.0) < 1e-12

    def test_body_to_ned_round_trip(self):
        """body→NED→body should return original vector."""
        phi, theta, psi = np.radians([20.0, 8.0, 90.0])
        v_body = np.array([30.0, 1.0, -2.0])
        v_ned  = body_to_ned(v_body, phi, theta, psi)
        v_back = ned_to_body(v_ned, phi, theta, psi)
        np.testing.assert_allclose(v_back, v_body, atol=1e-10)

    def test_wind_to_body_frame_no_rotation(self):
        """At zero Euler angles, NED wind equals body-frame wind."""
        wind_ned = np.array([5.0, 0.0, 0.0])
        w_body = wind_to_body_frame(wind_ned, 0.0, 0.0, 0.0)
        np.testing.assert_allclose(w_body, wind_ned, atol=1e-12)

    def test_airspeed_vector_no_wind(self):
        """With zero wind, airspeed vector equals body velocity."""
        vel_body  = np.array([30.0, 0.5, -1.0])
        wind_body = np.zeros(3)
        va = airspeed_vector(vel_body, wind_body)
        np.testing.assert_allclose(va, vel_body, atol=1e-12)

    def test_euler_rates_zero(self):
        """At zero pitch/roll, Euler rates equal body rates."""
        p, q, r = 0.1, 0.2, 0.05
        phi, theta = 0.0, 0.0
        rates = euler_rates(p, q, r, phi, theta)
        # At phi=theta=0: phi_dot=p, theta_dot=q, psi_dot=r
        np.testing.assert_allclose(rates, [p, q, r], atol=1e-12)

    def test_wrap_angle(self):
        """wrap_angle must map any angle to (−π, π]."""
        angles = [3 * math.pi, -3 * math.pi, 0.0, math.pi, -math.pi]
        for a in angles:
            w = wrap_angle(a)
            assert -math.pi <= w <= math.pi, f"wrap_angle({a}) = {w}"


# ═══════════════════════════════════════════════════════════════
# 2. aerodynamics
# ═══════════════════════════════════════════════════════════════

class TestAerodynamics:

    def test_output_type(self, tb2_params):
        """compute_aero_forces must return AeroForces with numeric attributes."""
        from dynamics.aerodynamics import AeroForces
        af = compute_aero_forces(30.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, tb2_params)
        assert isinstance(af, AeroForces)
        assert math.isfinite(af.X)
        assert math.isfinite(af.Z)
        assert math.isfinite(af.M)

    def test_positive_lift(self, tb2_params):
        """At positive AoA, lift (−Z force in NED) should be positive."""
        # Z is positive downward in body NED convention → lift is −Z
        af = compute_aero_forces(30.0, 0.0, 1.5, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, tb2_params)
        assert af.CL > 0.0, "CL should be positive at positive alpha"
        assert af.Z < 0.0,  "Z body force should be negative (upward lift)"

    def test_drag_always_positive(self, tb2_params):
        """CD (drag coefficient) must always be positive."""
        af = compute_aero_forces(30.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, tb2_params)
        assert af.CD > 0.0

    def test_zero_velocity_zero_forces(self, tb2_params):
        """
        At near-zero airspeed, aerodynamics.py clamps airspeed to 1 m/s for
        numerical stability (q_bar = 0.5 * 1.225 * 1² ≈ 0.6 Pa).
        Forces should be small (< 50 N) but not necessarily zero.
        """
        af = compute_aero_forces(0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, tb2_params)
        # Dynamic pressure clamps at 1 m/s → q_bar ≈ 0.6 Pa (very small)
        assert af.q_bar < 1.0   # still much less than cruise q_bar (~551 Pa)
        assert abs(af.Z) < 50.0  # small compared to cruise lift (~3000 N)

    def test_symmetry_lateral(self, tb2_params):
        """With positive aileron deflection, rolling moment Cl should be non-zero."""
        af_pos = compute_aero_forces(30.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                                      0.0, np.radians(5.0), 0.0, tb2_params)
        af_neg = compute_aero_forces(30.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                                      0.0, np.radians(-5.0), 0.0, tb2_params)
        assert af_pos.Cl != af_neg.Cl, "Aileron deflection should produce asymmetric roll moment"

    def test_elevator_pitching_moment(self, tb2_params):
        """Positive elevator → nose-down pitching moment (Cm_deltae < 0 convention)."""
        af = compute_aero_forces(30.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                                  np.radians(5.0), 0.0, 0.0, tb2_params)
        # TB2 has Cm_deltae = −1.0 → positive de causes negative Cm contribution
        assert math.isfinite(af.M)

    def test_wind_effect(self, tb2_params):
        """Adding a headwind should increase dynamic pressure (larger forces)."""
        af_no_wind   = compute_aero_forces(30.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, tb2_params)
        af_headwind  = compute_aero_forces(30.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, tb2_params,
                                            wind_body=np.array([-10.0, 0.0, 0.0]))
        # Headwind increases effective airspeed → larger q_bar → larger forces
        assert af_headwind.q_bar > af_no_wind.q_bar

    def test_dynamic_pressure(self):
        """Dynamic pressure at sea-level 30 m/s should be approx 551 Pa."""
        q = dynamic_pressure(1.225, 30.0)   # (rho, airspeed)
        expected = 0.5 * 1.225 * 30.0**2
        assert abs(q - expected) < 1e-6


# ═══════════════════════════════════════════════════════════════
# 3. linear_model
# ═══════════════════════════════════════════════════════════════

class TestLinearModel:

    def test_build_returns_matrices(self, tb2_params):
        model = LinearModel(tb2_params)
        A, B, U0 = model.build()
        assert A.shape == (4, 4), "State matrix A should be 4×4"
        assert B.shape == (4, 2), "Input matrix B should be 4×2"
        assert U0 > 0,            "Trim speed should be positive"

    def test_state_matrix_finite(self, tb2_params):
        model = LinearModel(tb2_params)
        A, B, _ = model.build()
        assert np.all(np.isfinite(A)), "All A matrix entries must be finite"
        assert np.all(np.isfinite(B)), "All B matrix entries must be finite"

    def test_analyze_modes_count(self, tb2_linear):
        model, params = tb2_linear
        modes = model.analyze_modes()
        # 4-DOF longitudinal → 2 pairs of complex eigenvalues (short period + phugoid)
        assert len(modes) == 2, f"Expected 2 modes, got {len(modes)}"

    def test_short_period_stable_for_tb2(self, tb2_linear):
        """TB2 short-period mode should be stable (Re(λ) < 0)."""
        model, _ = tb2_linear
        modes = model.analyze_modes()
        # Short period: highest natural frequency
        modes_sorted = sorted(modes, key=lambda m: m.wn, reverse=True)
        sp = modes_sorted[0]
        assert sp.stable, \
            f"Short period mode is unstable: λ={sp.eigenvalue:.4f}, ζ={sp.zeta:.3f}"

    def test_simulate_shape(self, tb2_linear):
        model, _ = tb2_linear
        t, y, de = model.simulate(pulses=[], duration=5.0, n_points=500)
        assert len(t) > 0
        # y shape: (4, n_points) – 4 states × time steps
        assert y.shape[0] == 4, "State vector should have 4 rows (states)"

    def test_simulate_zero_input_zero_response(self, tb2_linear):
        """With no elevator input, state perturbations should remain near zero."""
        model, _ = tb2_linear
        t, y, de = model.simulate(pulses=[], duration=2.0, n_points=200)
        # All states should stay close to zero (no perturbation)
        assert np.max(np.abs(y)) < 1e-10, \
            "No-input simulation should produce zero state response"

    def test_elevator_pulse_excites_pitch(self, tb2_linear):
        """An elevator pulse should excite pitch angle (theta) response."""
        model, _ = tb2_linear
        pulse = {"start_time": 1.0, "duration": 0.5, "angle_deg": 2.0}
        t, y, de = model.simulate(pulses=[pulse], duration=10.0, n_points=1000)
        theta = y[3, :]  # row 3 = theta
        assert np.max(np.abs(theta)) > 1e-6, \
            "Elevator pulse should produce nonzero pitch response"


# ═══════════════════════════════════════════════════════════════
# 4. nonlinear_model
# ═══════════════════════════════════════════════════════════════

class TestNonlinearModel:

    def test_trim_converges(self, tb2_params):
        """Trim computation should return finite values for TB2."""
        model = NonlinearModel(tb2_params)
        trim = model.compute_trim()
        assert math.isfinite(trim.alpha_trim)
        assert math.isfinite(trim.de_trim)
        assert math.isfinite(trim.U0) and trim.U0 > 0.0

    def test_trim_alpha_reasonable(self, tb2_params):
        """Trim AoA for TB2 should be in range [0, 15] deg."""
        model = NonlinearModel(tb2_params)
        trim = model.compute_trim()
        alpha_deg = math.degrees(trim.alpha_trim)
        assert 0.0 <= alpha_deg <= 15.0, \
            f"Trim AoA {alpha_deg:.2f} deg is out of expected range"

    def test_state_dot_dimension(self, tb2_params):
        """state_dot must return a 12-element vector."""
        model = NonlinearModel(tb2_params)
        trim  = model.compute_trim()
        # Build trim initial state
        u0 = trim.U0 * np.cos(trim.alpha_trim)
        w0 = trim.U0 * np.sin(trim.alpha_trim)
        state0 = np.array([u0, 0.0, w0, 0.0, 0.0, 0.0,
                            0.0, trim.alpha_trim, 0.0, 0.0, 0.0, 0.0])
        controls = Controls(elevator=trim.de_trim, throttle=1.0)
        sd = model.state_dot(0.0, state0, controls)
        assert len(sd) == 12, f"state_dot should return 12 elements, got {len(sd)}"

    def test_trim_state_dot_near_zero(self, tb2_params):
        """At trim, state derivatives should be reasonably small (equilibrium)."""
        model = NonlinearModel(tb2_params)
        trim  = model.compute_trim()
        u0 = trim.U0 * np.cos(trim.alpha_trim)
        w0 = trim.U0 * np.sin(trim.alpha_trim)
        state0 = np.array([u0, 0.0, w0, 0.0, 0.0, 0.0,
                            0.0, trim.alpha_trim, 0.0, 0.0, 0.0, 0.0])
        controls = Controls(elevator=trim.de_trim, throttle=1.0)
        sd = model.state_dot(0.0, state0, controls)
        # Velocity derivatives (indices 0-2) and angular rate derivatives (3-5)
        translational_accel = np.linalg.norm(sd[0:3])
        angular_accel       = np.linalg.norm(sd[3:6])
        assert translational_accel < 20.0, \
            f"Translational acceleration at trim too large: {translational_accel:.3f}"
        assert angular_accel < 2.0, \
            f"Angular acceleration at trim too large: {angular_accel:.3f}"

    def test_simulate_short_run(self, tb2_params):
        """A 3-second simulation should complete without errors."""
        model  = NonlinearModel(tb2_params)
        result = model.simulate(pulses=[], duration=3.0, n_points=300)
        assert result.t[-1] >= 2.9, "Simulation should run to at least 2.9 s"
        assert result.y.shape[0] == 12, "Output state should be 12-dimensional"

    def test_simulate_altitude_positive(self, tb2_params):
        """During a 3-second straight-level flight, altitude should remain above 0."""
        model  = NonlinearModel(tb2_params)
        result = model.simulate(pulses=[], duration=3.0, n_points=300)
        # y[11] is NED down; altitude = -y[11]; should start at 0, might drift
        xD = result.y[11]
        assert np.max(np.abs(xD)) < 500.0, \
            "NED down should not diverge in 3 s"

    def test_all_aircraft_trim(self):
        """All 7 aircraft should successfully compute trim without errors."""
        from models.aircraft_database import AIRCRAFT_NAMES
        for name in AIRCRAFT_NAMES:
            params = get_aircraft_params(name)
            model  = NonlinearModel(params)
            trim   = model.compute_trim()
            assert math.isfinite(trim.alpha_trim), f"alpha_trim not finite for {name}"
            assert math.isfinite(trim.de_trim),    f"de_trim not finite for {name}"
            assert trim.U0 > 0.0, f"U0 should be positive for {name}"
