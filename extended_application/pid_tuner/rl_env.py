"""
rl_env.py  –  Gymnasium-compatible PID auto-tuning environment.

The environment wraps a lightweight PID step-response simulation
(NOT the full 6-DOF sim, for speed) and exposes a Gym interface so
that any RL algorithm (PPO, DDPG, SAC …) can learn optimal PID gains.

State space (observation):
    [e(t), e_dot(t), e_int(t),          # error, derivative, integral
     kp, ki, kd,                         # current gains (normalised 0-1)
     overshoot, settling_time_est]        # performance metrics (running)
    → shape (9,)

Action space (continuous):
    [Δkp, Δki, Δkd]  ∈ [-1, 1]³  (incremental gain updates)

Reward:
    r = -|e| - λ_u * u²  - λ_os * max(0, overshoot-threshold)
    where u = PID output (control effort), overshoot is peak-error ratio.

Supported axes: "pitch" | "roll" | "yaw"

Usage
-----
from pid_tuner.rl_env import PIDTuningEnv

env = PIDTuningEnv(axis="pitch", dt=0.01, episode_steps=500)
obs, info = env.reset()
for _ in range(500):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
"""

from __future__ import annotations

import math
import numpy as np
from typing import Any, Dict, Optional, Tuple

try:
    import gymnasium as gym
    from gymnasium import spaces
    GYM_AVAILABLE = True
except ImportError:
    try:
        import gym
        from gym import spaces
        GYM_AVAILABLE = True
    except ImportError:
        GYM_AVAILABLE = False


# ---------------------------------------------------------------------------
# Minimal PID simulator (pure Python / NumPy, no 6-DOF overhead)
# ---------------------------------------------------------------------------

class _MiniPID:
    """Discrete PID with anti-windup for the RL environment."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.01,
                 out_min=-1.0, out_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self._integral = 0.0
        self._prev_err = 0.0
        self._saturated = False

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0
        self._saturated = False

    def step(self, error: float) -> float:
        d = (error - self._prev_err) / self.dt
        p_out = self.kp * error
        if not self._saturated:
            self._integral = np.clip(
                self._integral + self.ki * error * self.dt,
                self.out_min, self.out_max)
        raw = p_out + self._integral + self.kd * d
        out = float(np.clip(raw, self.out_min, self.out_max))
        self._saturated = (raw != out)
        self._prev_err = error
        return out


# ---------------------------------------------------------------------------
# First-order plant model (approximates pitch/roll/yaw dynamics)
# ---------------------------------------------------------------------------

class _FirstOrderPlant:
    """
    Discrete first-order plant:   τ·ẋ = -x + K·u
    → x[k+1] = x[k] + dt/τ·(-x[k] + K·u[k])

    Axis defaults tuned to approximate TB2 linearised modes:
      pitch : τ=0.8s, K=1.2
      roll  : τ=0.3s, K=1.8
      yaw   : τ=1.5s, K=0.6
    """

    _AXIS_PARAMS = {
        "pitch": {"tau": 0.8,  "K": 1.2},
        "roll":  {"tau": 0.3,  "K": 1.8},
        "yaw":   {"tau": 1.5,  "K": 0.6},
    }

    def __init__(self, axis: str = "pitch", dt: float = 0.01):
        p = self._AXIS_PARAMS.get(axis, self._AXIS_PARAMS["pitch"])
        self.tau = p["tau"]
        self.K   = p["K"]
        self.dt  = dt
        self.x   = 0.0

    def reset(self, x0: float = 0.0):
        self.x = x0

    def step(self, u: float) -> float:
        self.x += self.dt / self.tau * (-self.x + self.K * u)
        return self.x


# ---------------------------------------------------------------------------
# Cascaded two-loop plant (attitude outer + rate inner)
# ---------------------------------------------------------------------------

class _CascadedPlant:
    """
    Cascaded plant that mimics ArduPlane's attitude + rate control structure.

    Outer loop (attitude):
        attitude_ref  →  [kp_att, kd_att]  →  rate_ref
    Inner loop (rate):
        rate_ref  →  [kp_rate, ki_rate, kd_rate]  →  actuator_cmd  →  angular_rate
    Kinematics:
        attitude += angular_rate * dt

    Angular-rate dynamics (first-order):
        τ_rate · dω/dt = -ω + K_rate · actuator_cmd

    Axis parameters (approximate TB2 linearised modes):
      pitch : τ_rate=0.15s, K_rate=10.0
      roll  : τ_rate=0.08s, K_rate=14.0
      yaw   : τ_rate=0.30s, K_rate= 5.0
    """

    _AXIS_PARAMS = {
        "pitch": {"tau_rate": 0.15, "K_rate": 10.0},
        "roll":  {"tau_rate": 0.08, "K_rate": 14.0},
        "yaw":   {"tau_rate": 0.30, "K_rate":  5.0},
    }

    def __init__(self, axis: str = "pitch", dt: float = 0.01):
        p = self._AXIS_PARAMS.get(axis, self._AXIS_PARAMS["pitch"])
        self.tau_rate = p["tau_rate"]
        self.K_rate   = p["K_rate"]
        self.dt       = dt

        # Outer loop (attitude PD)
        self.kp_att = 1.0
        self.kd_att = 0.0
        self._att_prev_err = 0.0

        # Inner loop (rate PID)
        self.kp_rate = 0.1
        self.ki_rate = 0.0
        self.kd_rate = 0.0
        self._rate_pid = _MiniPID(kp=self.kp_rate, ki=self.ki_rate,
                                  kd=self.kd_rate, dt=dt,
                                  out_min=-1.0, out_max=1.0)

        # States
        self.attitude  = 0.0   # θ (rad, normalised)
        self.rate      = 0.0   # ω (rad/s, normalised)
        self.rate_ref  = 0.0   # inner-loop setpoint

    def reset(self) -> None:
        self.attitude  = 0.0
        self.rate      = 0.0
        self.rate_ref  = 0.0
        self._att_prev_err = 0.0
        self._rate_pid.reset()
        self._rate_pid.kp = self.kp_rate
        self._rate_pid.ki = self.ki_rate
        self._rate_pid.kd = self.kd_rate

    def set_att_gains(self, kp: float, kd: float = 0.0) -> None:
        self.kp_att = kp
        self.kd_att = kd

    def set_rate_gains(self, kp: float, ki: float, kd: float) -> None:
        self.kp_rate = kp
        self.ki_rate = ki
        self.kd_rate = kd
        self._rate_pid.kp = kp
        self._rate_pid.ki = ki
        self._rate_pid.kd = kd

    def step(self, att_ref: float) -> None:
        """Advance one time step given an attitude setpoint."""
        # Outer PD: attitude error → rate command
        att_err = att_ref - self.attitude
        att_err_dot = (att_err - self._att_prev_err) / self.dt
        self._att_prev_err = att_err
        self.rate_ref = float(np.clip(
            self.kp_att * att_err + self.kd_att * att_err_dot,
            -5.0, 5.0))

        # Inner PID: rate error → actuator
        rate_err = self.rate_ref - self.rate
        actuator = self._rate_pid.step(rate_err)

        # First-order rate dynamics
        self.rate += self.dt / self.tau_rate * (-self.rate + self.K_rate * actuator)

        # Kinematics: integrate rate → attitude
        self.attitude += self.rate * self.dt


# ---------------------------------------------------------------------------
# Reference trajectory generator
# ---------------------------------------------------------------------------

def _step_setpoint(t: float, amplitude: float = 1.0,
                   start: float = 0.5, end: float = 999.0) -> float:
    """Simple step reference."""
    return amplitude if start <= t < end else 0.0


def _doublet_setpoint(t: float, amplitude: float = 1.0,
                      t1: float = 0.5, t2: float = 2.0, t3: float = 3.5) -> float:
    """Doublet (positive then negative step) – good for system identification."""
    if t1 <= t < t2:
        return amplitude
    elif t2 <= t < t3:
        return -amplitude
    return 0.0


# ---------------------------------------------------------------------------
# Gain normalisation helpers
# ---------------------------------------------------------------------------

# Normalised gain range for observation [0, 1]
_GAIN_RANGES = {
    "pitch": {"kp": (0.0, 5.0), "ki": (0.0, 2.0), "kd": (0.0, 0.5)},
    "roll":  {"kp": (0.0, 5.0), "ki": (0.0, 2.0), "kd": (0.0, 0.5)},
    "yaw":   {"kp": (0.0, 3.0), "ki": (0.0, 1.0), "kd": (0.0, 0.2)},
}


def _norm(val, lo, hi):
    return float(np.clip((val - lo) / max(hi - lo, 1e-9), 0.0, 1.0))


def _denorm(val_n, lo, hi):
    return lo + val_n * (hi - lo)


# ---------------------------------------------------------------------------
# PIDTuningEnv
# ---------------------------------------------------------------------------

class PIDTuningEnv:
    """
    Gymnasium-compatible PID gain tuning environment.

    Observation : (9,) float32
        [error, error_dot, error_int, kp_n, ki_n, kd_n,
         overshoot_n, settling_est_n, ref_value]

    Action      : (3,) float32  ∈ [-1, 1]
        Incremental gain changes [Δkp, Δki, Δkd] (scaled internally)

    Reward      : scalar
        −|error| − λ_effort·u² − λ_overshoot·max(0, overshoot − threshold)

    Parameters
    ----------
    axis            : "pitch" | "roll" | "yaw"
    dt              : time step (s)
    episode_steps   : maximum steps per episode
    ref_type        : "step" | "doublet"
    ref_amplitude   : setpoint amplitude (rad or normalised)
    lambda_effort   : penalty weight on control effort
    lambda_overshoot: penalty weight on overshoot
    overshoot_thresh: acceptable overshoot fraction (0.1 = 10 %)
    delta_gain_scale: max gain change per action step
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        axis:             str   = "pitch",
        dt:               float = 0.01,
        episode_steps:    int   = 500,
        ref_type:         str   = "step",
        ref_amplitude:    float = 1.0,
        lambda_effort:    float = 0.01,
        lambda_overshoot: float = 2.0,
        overshoot_thresh: float = 0.10,
        delta_gain_scale: float = 0.05,
    ):
        self.axis             = axis
        self.dt               = dt
        self.episode_steps    = episode_steps
        self.ref_type         = ref_type
        self.ref_amplitude    = ref_amplitude
        self.lambda_effort    = lambda_effort
        self.lambda_overshoot = lambda_overshoot
        self.overshoot_thresh = overshoot_thresh
        self.delta_gain_scale = delta_gain_scale

        self._gr = _GAIN_RANGES.get(axis, _GAIN_RANGES["pitch"])

        # Cascaded plant (attitude outer + rate inner)
        self._plant = _CascadedPlant(axis=axis, dt=dt)

        # RL inner-loop PID proxy (used only for RL gain update bookkeeping)
        self._pid = _MiniPID(dt=dt)

        # Gym spaces
        obs_low  = np.full(9, -10.0, dtype=np.float32)
        obs_high = np.full(9,  10.0, dtype=np.float32)
        obs_low [3:8] = 0.0
        obs_high[3:8] = 1.0

        if GYM_AVAILABLE:
            self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)
            self.action_space      = spaces.Box(
                low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Outer-loop attitude gains (set externally by GUI or RL)
        self._kp_att = 1.0
        self._kd_att = 0.0

        # Inner-loop rate gains (RL tunes these)
        self._kp      = (_GAIN_RANGES[axis]["kp"][0] + _GAIN_RANGES[axis]["kp"][1]) / 2
        self._ki      = 0.0
        self._kd      = 0.0

        # Metrics
        self._step     = 0
        self._t        = 0.0
        self._err_int  = 0.0
        self._prev_err = 0.0
        self._peak_err = 0.0
        self._settled  = False
        self._settle_t = episode_steps * dt

    # ------------------------------------------------------------------
    # Gym API
    # ------------------------------------------------------------------

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        if seed is not None:
            np.random.seed(seed)

        self._pid.reset()
        self._plant.set_att_gains(self._kp_att, self._kd_att)
        self._plant.set_rate_gains(self._kp, self._ki, self._kd)
        self._plant.reset()

        # Randomise starting gains slightly around defaults ONLY for RL training.
        # When called from the GUI worker the caller restores its own gains
        # immediately after reset(), so we only randomise if no gains have been
        # set externally (i.e. kp is still the init-time mid-range default).
        _kp_mid = (self._gr["kp"][0] + self._gr["kp"][1]) / 2
        if self._kp == _kp_mid and self._ki == 0.0 and self._kd == 0.0:
            # RL training path: randomise
            self._kp = float(np.random.uniform(
                self._gr["kp"][0], self._gr["kp"][1] * 0.4))
            self._ki = float(np.random.uniform(0.0, self._gr["ki"][1] * 0.1))
            self._kd = float(np.random.uniform(0.0, self._gr["kd"][1] * 0.1))
        # else: keep the gains that were set externally (GUI / ParamStore)
        self._pid.kp, self._pid.ki, self._pid.kd = self._kp, self._ki, self._kd
        self._plant.set_rate_gains(self._kp, self._ki, self._kd)

        self._step     = 0
        self._t        = 0.0
        self._err_int  = 0.0
        self._prev_err = 0.0
        self._peak_err = 0.0
        self._settled  = False
        self._settle_t = self.episode_steps * self.dt

        obs = self._get_obs(0.0, 0.0, 0.0)
        return obs, {}

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        action = np.clip(action, -1.0, 1.0)

        # --- Update gains via incremental action ----------------------------
        dkp = float(action[0]) * self.delta_gain_scale * (self._gr["kp"][1] - self._gr["kp"][0])
        dki = float(action[1]) * self.delta_gain_scale * (self._gr["ki"][1] - self._gr["ki"][0])
        dkd = float(action[2]) * self.delta_gain_scale * (self._gr["kd"][1] - self._gr["kd"][0])

        self._kp = float(np.clip(self._kp + dkp, *self._gr["kp"]))
        self._ki = float(np.clip(self._ki + dki, *self._gr["ki"]))
        self._kd = float(np.clip(self._kd + dkd, *self._gr["kd"]))
        self._pid.kp, self._pid.ki, self._pid.kd = self._kp, self._ki, self._kd
        self._plant.set_rate_gains(self._kp, self._ki, self._kd)

        # --- Cascaded plant step -------------------------------------------
        ref = self._get_ref(self._t)
        self._plant.step(ref)                  # advances attitude + rate
        y   = self._plant.attitude             # outer output (attitude)
        err = ref - y                          # attitude tracking error
        u   = self._plant.rate                 # inner output (rate) – proxy effort

        # --- Performance metrics -------------------------------------------
        self._err_int  += abs(err) * self.dt
        self._prev_err  = err
        if ref != 0.0:
            self._peak_err = max(self._peak_err, abs(err) / abs(ref))

        # Settling detection (within 5% of setpoint, held 0.2 s)
        if not self._settled and ref != 0.0:
            within = abs(err) / abs(ref) < 0.05
            if within and not self._settled:
                self._settled = True
                self._settle_t = self._t

        # --- Reward ---------------------------------------------------------
        overshoot = max(0.0, self._peak_err - self.overshoot_thresh)
        reward = (
            - abs(err)
            - self.lambda_effort    * u**2
            - self.lambda_overshoot * overshoot
        )

        # --- Advance time ---------------------------------------------------
        self._step += 1
        self._t    += self.dt

        terminated = False
        truncated  = (self._step >= self.episode_steps)

        obs  = self._get_obs(err, u, ref)
        info = {
            "kp": self._kp, "ki": self._ki, "kd": self._kd,
            "error":    err,
            "output":   y,                          # attitude (outer loop output)
            "ref":      ref,                        # attitude setpoint
            "rate":     self._plant.rate,           # angular rate (inner loop output)
            "rate_ref": self._plant.rate_ref,       # rate setpoint (outer → inner)
            "peak_overshoot": self._peak_err,
            "settle_time": self._settle_t,
            "integral_abs_error": self._err_int,
        }
        return obs, float(reward), terminated, truncated, info

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_ref(self, t: float) -> float:
        if self.ref_type == "doublet":
            return _doublet_setpoint(t, self.ref_amplitude)
        return _step_setpoint(t, self.ref_amplitude)

    def _get_obs(self, err: float, u: float, ref: float) -> np.ndarray:
        gr = self._gr
        err_dot = (err - self._prev_err) / self.dt if self._step > 0 else 0.0
        obs = np.array([
            np.clip(err,       -5.0, 5.0),
            np.clip(err_dot,  -50.0, 50.0),
            np.clip(self._err_int, 0.0, 10.0),
            _norm(self._kp, *gr["kp"]),
            _norm(self._ki, *gr["ki"]),
            _norm(self._kd, *gr["kd"]),
            np.clip(self._peak_err, 0.0, 1.0),
            _norm(self._settle_t, 0.0, self.episode_steps * self.dt),
            np.clip(ref, -2.0, 2.0),
        ], dtype=np.float32)
        return obs

    def render(self, mode="human"):
        pass

    def close(self):
        pass

    # ------------------------------------------------------------------
    # Property: current gains as dict (for ParamStore integration)
    # ------------------------------------------------------------------

    @property
    def current_gains(self) -> Dict[str, float]:
        """Map current gains to ArdupilotParams key names."""
        mapping = {
            "pitch": {
                "kp": "PTCH_RATE_P",
                "ki": "PTCH_RATE_I",
                "kd": "PTCH_RATE_D",
            },
            "roll": {
                "kp": "ROLL_RATE_P",
                "ki": "ROLL_RATE_I",
                "kd": "ROLL_D",
            },
            "yaw": {
                "kp": "YAW_RATE_P",
                "ki": "YAW_RATE_I",
                "kd": "YAW_P",
            },
        }
        m = mapping.get(self.axis, mapping["pitch"])
        return {m["kp"]: self._kp, m["ki"]: self._ki, m["kd"]: self._kd}


# ---------------------------------------------------------------------------
# Convenience factory
# ---------------------------------------------------------------------------

def make_env(axis: str = "pitch", **kwargs: Any) -> "PIDTuningEnv":
    """Create a PIDTuningEnv. Raises ImportError if gymnasium/gym unavailable."""
    if not GYM_AVAILABLE:
        raise ImportError(
            "gymnasium or gym is required. Install with: pip install gymnasium")
    return PIDTuningEnv(axis=axis, **kwargs)
