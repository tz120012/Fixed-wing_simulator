"""
test_control.py  –  Unit tests for control/ module.

Covers:
  - PIDController : proportional/integral/derivative, anti-windup, reset, gains
  - ArdupilotParams : defaults, from_dict, validate, LIM_ROLL_DEG property
  - AttitudeController : output structure, sign conventions
  - RateController : output structure, zero error → zero output
  - ServoMixer : amplitude limits, to_radians, coordinated turn rudder
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

from control.pid_controller    import PIDController
from control.ardupilot_compat  import ArdupilotParams
from control.attitude_controller import AttitudeController, AttitudeOutput
from control.rate_controller     import RateController, RateOutput
from control.servo_mixer         import ServoMixer, ServoOutput


# ──────────────────────────────────────────────────────────────
# Shared fixture
# ──────────────────────────────────────────────────────────────
@pytest.fixture
def default_ap():
    return ArdupilotParams()


@pytest.fixture
def attitude_ctrl(default_ap):
    return AttitudeController(default_ap, dt=0.01)


@pytest.fixture
def rate_ctrl(default_ap):
    return RateController(default_ap, dt=0.01)


@pytest.fixture
def mixer(default_ap):
    return ServoMixer(default_ap, dt=0.01)


# ═══════════════════════════════════════════════════════════════
# 1. PIDController
# ═══════════════════════════════════════════════════════════════

class TestPIDController:

    def test_pure_proportional(self):
        """P-only controller: output = kp × error."""
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0,
                             output_min=-100.0, output_max=100.0)
        out = pid.update(error=3.0, dt=0.01)
        assert abs(out - 6.0) < 1e-10

    def test_integral_accumulates(self):
        """Integral term should accumulate over multiple steps."""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0,
                             output_min=-1000.0, output_max=1000.0)
        for _ in range(10):
            pid.update(error=1.0, dt=0.1)   # each step: Δintegral = ki*e*dt = 0.1
        # After 10 steps: integral ≈ 1.0
        out = pid.update(error=0.0, dt=0.1)
        assert abs(out - 1.0) < 1e-6

    def test_derivative_first_step(self):
        """On first step: derivative = (error - 0) / dt = error / dt."""
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0,
                             output_min=-1000.0, output_max=1000.0,
                             d_lpf_hz=0.0)    # disable LPF for clean test
        out = pid.update(error=1.0, dt=0.01)
        # d_raw = 1.0 / 0.01 = 100, kd * d_raw = 100
        assert abs(out - 100.0) < 1.0   # allow small LPF residual

    def test_output_saturated_at_max(self):
        """Output must be clamped to output_max."""
        pid = PIDController(kp=100.0, output_min=-1.0, output_max=1.0)
        out = pid.update(error=10.0, dt=0.01)
        assert out == pytest.approx(1.0)

    def test_output_saturated_at_min(self):
        """Output must be clamped to output_min."""
        pid = PIDController(kp=100.0, output_min=-1.0, output_max=1.0)
        out = pid.update(error=-10.0, dt=0.01)
        assert out == pytest.approx(-1.0)

    def test_anti_windup_stops_integration(self):
        """When output is saturated, integral should not keep growing (clamping)."""
        pid = PIDController(kp=10.0, ki=10.0, kd=0.0,
                             output_min=-1.0, output_max=1.0)
        # Drive output to saturation
        for _ in range(100):
            pid.update(error=1.0, dt=0.01)
        integral_after_windup = pid._integral
        # Apply a few more steps – integral should not grow significantly
        for _ in range(100):
            pid.update(error=1.0, dt=0.01)
        assert pid._integral <= integral_after_windup + 0.01, \
            "Integral should not grow when saturated (anti-windup failed)"

    def test_reset_clears_state(self):
        """reset() should zero the integral, derivative, and saturated flag."""
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0,
                             output_min=-100.0, output_max=100.0)
        for _ in range(50):
            pid.update(error=1.0, dt=0.01)
        pid.reset()
        assert pid._integral    == 0.0
        assert pid._prev_error  == 0.0
        assert pid._d_filtered  == 0.0
        assert pid._saturated   is False

    def test_set_gains(self):
        """set_gains() should update only the specified gains."""
        pid = PIDController(kp=1.0, ki=0.5, kd=0.1)
        pid.set_gains(kp=2.0)
        assert pid.kp == 2.0
        assert pid.ki == 0.5   # unchanged
        assert pid.kd == 0.1   # unchanged

    def test_feed_forward_added(self):
        """Feed-forward term should be added to the output."""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0,
                             output_min=-100.0, output_max=100.0)
        out = pid.update(error=0.0, dt=0.01, feed_forward=3.0)
        assert abs(out - 3.0) < 1e-10

    def test_zero_error_zero_output_fresh(self):
        """Fresh controller with zero error should produce zero output."""
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0,
                             output_min=-100.0, output_max=100.0)
        out = pid.update(error=0.0, dt=0.01)
        assert abs(out) < 1e-12


# ═══════════════════════════════════════════════════════════════
# 2. ArdupilotParams
# ═══════════════════════════════════════════════════════════════

class TestArdupilotParams:

    def test_default_values(self, default_ap):
        assert default_ap.PTCH_P     == pytest.approx(1.0)
        assert default_ap.ROLL_P     == pytest.approx(1.0)
        assert default_ap.YAW_RATE_P == pytest.approx(0.02)
        assert default_ap.LIM_PITCH_MAX == pytest.approx(20.0)

    def test_lim_roll_deg_property(self, default_ap):
        """LIM_ROLL_DEG = LIM_ROLL_CD / 100."""
        expected = default_ap.LIM_ROLL_CD / 100.0
        assert default_ap.LIM_ROLL_DEG == pytest.approx(expected)

    def test_from_dict_partial(self):
        """from_dict with partial keys should fill the rest with defaults."""
        ap = ArdupilotParams.from_dict({"PTCH_P": 2.0, "UNKNOWN_KEY": 99.0})
        assert ap.PTCH_P == pytest.approx(2.0)
        assert ap.ROLL_P == pytest.approx(1.0)   # default

    def test_from_dict_unknown_keys_ignored(self):
        """Unknown keys should not raise an exception."""
        ap = ArdupilotParams.from_dict({"NOT_A_PARAM": 123.0})
        assert ap.PTCH_P == pytest.approx(1.0)

    def test_validate_passes_defaults(self, default_ap):
        """Default parameters should pass validation."""
        result = default_ap.validate()
        assert result is True, "Validation should return True for default params"

    def test_validate_catches_negative_kp(self):
        """Negative PTCH_P should fail validation."""
        ap = ArdupilotParams(PTCH_P=-1.0)
        result = ap.validate()
        assert result is False

    def test_to_dict_roundtrip(self, default_ap):
        """to_dict → from_dict should reproduce the same parameters."""
        d  = default_ap.to_dict()
        ap2 = ArdupilotParams.from_dict(d)
        assert ap2.PTCH_P     == pytest.approx(default_ap.PTCH_P)
        assert ap2.PTCH_RATE_D == pytest.approx(default_ap.PTCH_RATE_D)


# ═══════════════════════════════════════════════════════════════
# 3. AttitudeController
# ═══════════════════════════════════════════════════════════════

class TestAttitudeController:

    def test_output_type(self, attitude_ctrl):
        out = attitude_ctrl.update(
            phi=0.0, theta=0.0, psi=0.0,
            roll_cmd=0.0, pitch_cmd=0.0, yaw_cmd=0.0,
        )
        assert isinstance(out, AttitudeOutput)

    def test_zero_error_zero_rates(self, attitude_ctrl):
        """With zero error, desired angular rates should be zero."""
        out = attitude_ctrl.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert abs(out.roll_rate_cmd)  < 1e-10
        assert abs(out.pitch_rate_cmd) < 1e-10
        assert abs(out.yaw_rate_cmd)   < 1e-10

    def test_pitch_error_produces_pitch_rate(self, attitude_ctrl):
        """Pitch angle error → nonzero desired pitch rate."""
        out = attitude_ctrl.update(
            phi=0.0, theta=0.0,    psi=0.0,
            roll_cmd=0.0, pitch_cmd=np.radians(5.0), yaw_cmd=0.0,
        )
        assert abs(out.pitch_rate_cmd) > 0.0

    def test_roll_error_produces_roll_rate(self, attitude_ctrl):
        out = attitude_ctrl.update(
            phi=0.0, theta=0.0, psi=0.0,
            roll_cmd=np.radians(10.0), pitch_cmd=0.0, yaw_cmd=0.0,
        )
        assert abs(out.roll_rate_cmd) > 0.0

    def test_output_within_rate_limits(self, attitude_ctrl):
        """Desired rates must not exceed physical limits."""
        out = attitude_ctrl.update(
            phi=np.radians(-45.0), theta=np.radians(-20.0), psi=0.0,
            roll_cmd=np.radians(45.0), pitch_cmd=np.radians(20.0), yaw_cmd=np.radians(10.0),
        )
        MAX_ROLL  = np.radians(120.0)
        MAX_PITCH = np.radians(60.0)
        MAX_YAW   = np.radians(45.0)
        assert abs(out.roll_rate_cmd)  <= MAX_ROLL  + 1e-9
        assert abs(out.pitch_rate_cmd) <= MAX_PITCH + 1e-9
        assert abs(out.yaw_rate_cmd)   <= MAX_YAW   + 1e-9

    def test_pitch_sign_convention(self, attitude_ctrl):
        """Positive pitch command when current pitch < command → positive rate."""
        out = attitude_ctrl.update(
            phi=0.0, theta=0.0, psi=0.0,
            roll_cmd=0.0, pitch_cmd=np.radians(5.0), yaw_cmd=0.0,
        )
        assert out.pitch_rate_cmd > 0.0, \
            "Positive pitch error should produce positive pitch rate command"


# ═══════════════════════════════════════════════════════════════
# 4. RateController
# ═══════════════════════════════════════════════════════════════

class TestRateController:

    def test_output_type(self, rate_ctrl):
        out = rate_ctrl.update(p=0.0, q=0.0, r=0.0,
                                p_cmd=0.0, q_cmd=0.0, r_cmd=0.0)
        assert isinstance(out, RateOutput)

    def test_zero_error_zero_output(self, rate_ctrl):
        """No rate error → zero surface deflection increments."""
        out = rate_ctrl.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert abs(out.elevator) < 1e-10
        assert abs(out.aileron)  < 1e-10
        assert abs(out.rudder)   < 1e-10

    def test_pitch_rate_error_elevator(self, rate_ctrl):
        """Pitch rate error → nonzero elevator increment."""
        out = rate_ctrl.update(p=0.0, q=0.0, r=0.0,
                                p_cmd=0.0, q_cmd=np.radians(5.0), r_cmd=0.0)
        assert abs(out.elevator) > 0.0

    def test_roll_rate_error_aileron(self, rate_ctrl):
        """Roll rate error → nonzero aileron increment."""
        out = rate_ctrl.update(p=0.0, q=0.0, r=0.0,
                                p_cmd=np.radians(10.0), q_cmd=0.0, r_cmd=0.0)
        assert abs(out.aileron) > 0.0

    def test_output_clamped(self, rate_ctrl):
        """Large rate error should produce clamped output in [−1, 1]."""
        out = rate_ctrl.update(
            p=0.0, q=0.0, r=0.0,
            p_cmd=np.radians(1000.0), q_cmd=np.radians(1000.0), r_cmd=np.radians(1000.0),
        )
        assert -1.0 <= out.elevator <= 1.0
        assert -1.0 <= out.aileron  <= 1.0
        assert -1.0 <= out.rudder   <= 1.0

    def test_reset_clears_integrators(self, rate_ctrl):
        """reset() should zero all integrators in the rate PIDs."""
        for _ in range(50):
            rate_ctrl.update(p=0.1, q=0.2, r=0.05,
                              p_cmd=0.0, q_cmd=0.0, r_cmd=0.0)
        rate_ctrl.reset()
        out_after = rate_ctrl.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # After reset, pure proportional + integral = 0 (fresh state)
        assert abs(out_after.elevator) < 1e-9
        assert abs(out_after.aileron)  < 1e-9


# ═══════════════════════════════════════════════════════════════
# 5. ServoMixer
# ═══════════════════════════════════════════════════════════════

class TestServoMixer:

    def test_output_type(self, mixer):
        out = mixer.update(0.0, 0.0, 0.0, 0.5, phi=0.0, p=0.0)
        assert isinstance(out, ServoOutput)

    def test_zero_increments_zero_surfaces(self, mixer):
        """Zero increments → zero surface deflections (minus rate-limit warm-up)."""
        out = mixer.update(0.0, 0.0, 0.0, 0.5, phi=0.0, p=0.0)
        # elevator and aileron may be limited by rate-limiter from initial state 0
        assert abs(out.elevator) < 1e-6
        assert abs(out.aileron)  < 1e-6

    def test_throttle_clamped(self, mixer):
        """Throttle must stay in [THR_MIN, THR_MAX] = [0, 1]."""
        out_high = mixer.update(0.0, 0.0, 0.0, 2.0, phi=0.0, p=0.0)
        out_low  = mixer.update(0.0, 0.0, 0.0, -1.0, phi=0.0, p=0.0)
        assert out_high.throttle <= 1.0 + 1e-12
        assert out_low.throttle  >= 0.0 - 1e-12

    def test_elevator_amplitude_limit(self, mixer):
        """Elevator must not exceed normalised LIM_PITCH_MAX / 25 deg."""
        ap     = mixer.ap
        elev_max = math.radians(ap.LIM_PITCH_MAX) / math.radians(25.0)
        # Feed a large increment; allow multiple steps for rate limiter to ramp up
        for _ in range(200):
            out = mixer.update(1.0, 0.0, 0.0, 0.5, phi=0.0, p=0.0)
        assert out.elevator <= elev_max + 1e-9

    def test_to_radians(self, mixer):
        """to_radians converts normalised outputs to V-tail deflections (V-tail config)."""
        out = ServoOutput(elevator=1.0, aileron=-1.0, rudder=0.5, throttle=0.8)
        da, dv_left, dv_right, throttle = out.to_radians(
            elev_max_rad=math.radians(25.0),
            ail_max_rad=math.radians(20.0),
            rud_max_rad=math.radians(25.0),
            vtail_max_rad=math.radians(25.0),
        )
        # Aileron should be negative
        assert da == pytest.approx(-math.radians(20.0))
        
        # V-tail mixing: de_virtual = -1.0 * 25° = -25° (nose down)
        #                dr_virtual =  0.5 * 25° =  12.5° (nose right)
        # dv_left  = de_virtual - dr_virtual = -25° - 12.5° = -37.5° (saturated to -25°)
        # dv_right = de_virtual + dr_virtual = -25° + 12.5° = -12.5°
        assert dv_left  == pytest.approx(-math.radians(25.0))  # saturated
        assert dv_right == pytest.approx(-math.radians(12.5))
        assert throttle == 0.8

    def test_coordinated_turn_rudder(self, mixer):
        """Non-zero roll rate should produce rudder compensation."""
        out_no_roll   = mixer.update(0.0, 0.0, 0.0, 0.5, phi=0.0, p=0.0)
        out_roll_rate = mixer.update(0.0, 0.0, 0.0, 0.5, phi=0.0, p=np.radians(20.0))
        # Coordinated turn: rudder should differ
        assert out_roll_rate.rudder != out_no_roll.rudder, \
            "Non-zero roll rate should produce different rudder output (coord. turn)"

    def test_outputs_normalised(self, mixer):
        """All normalised outputs must stay within [-1, 1]."""
        for _ in range(50):
            out = mixer.update(
                elev_in=0.5, ail_in=-0.5, rud_in=0.3,
                throttle=0.7, phi=np.radians(15.0), p=np.radians(10.0)
            )
        assert -1.0 <= out.elevator <= 1.0
        assert -1.0 <= out.aileron  <= 1.0
        assert -1.0 <= out.rudder   <= 1.0
        assert  0.0 <= out.throttle <= 1.0
