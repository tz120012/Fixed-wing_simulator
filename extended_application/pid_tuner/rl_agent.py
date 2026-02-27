"""
rl_agent.py  –  Lightweight PPO agent for PID gain tuning.

Implements Proximal Policy Optimization (PPO-clip) with:
  - Shared actor-critic MLP backbone
  - GAE (Generalised Advantage Estimation)
  - Entropy bonus for exploration
  - No external RL library required (pure NumPy + optional torch/scipy)

If PyTorch is available → uses torch.nn for the policy.
If not              → falls back to a simple linear policy (scipy.optimize).

Usage
-----
from pid_tuner.rl_agent import PPOAgent, train, load_gains_from_checkpoint

# Train
agent = train(axis="pitch", total_steps=50_000, save_path="pid_tuner/checkpoints/pitch_ppo.npz")

# Inference: get optimal gains for a given observation
gains = agent.suggest_gains(obs)

# Load pre-trained checkpoint and push to ParamStore
from pid_tuner.param_store import ParamStore
store = ParamStore()
load_gains_from_checkpoint("pid_tuner/checkpoints/pitch_ppo.npz", store, axis="pitch")
"""

from __future__ import annotations

import os
import time
import math
import numpy as np
from typing import Dict, List, Optional, Tuple

from pid_tuner.rl_env import PIDTuningEnv, _GAIN_RANGES


# ---------------------------------------------------------------------------
# Try importing torch; fall back to numpy-only linear policy
# ---------------------------------------------------------------------------

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.distributions import Normal
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


# ===========================================================================
# PyTorch Actor-Critic Policy
# ===========================================================================

if TORCH_AVAILABLE:
    class _ActorCritic(nn.Module):
        """Shared-trunk MLP: obs → (action_mean, log_std, value)."""

        def __init__(self, obs_dim: int = 9, act_dim: int = 3,
                     hidden: int = 128):
            super().__init__()
            self.trunk = nn.Sequential(
                nn.Linear(obs_dim, hidden), nn.Tanh(),
                nn.Linear(hidden, hidden),  nn.Tanh(),
            )
            self.actor_mean = nn.Linear(hidden, act_dim)
            self.actor_logstd = nn.Parameter(torch.zeros(act_dim))
            self.critic = nn.Linear(hidden, 1)

        def forward(self, obs: torch.Tensor):
            h = self.trunk(obs)
            mean   = torch.tanh(self.actor_mean(h))   # clipped to [-1, 1]
            logstd = self.actor_logstd.expand_as(mean)
            value  = self.critic(h).squeeze(-1)
            return mean, logstd, value

        def get_dist(self, obs: torch.Tensor) -> "Normal":
            mean, logstd, _ = self.forward(obs)
            return Normal(mean, logstd.exp().clamp(1e-4, 1.0))


# ===========================================================================
# PPOAgent (PyTorch backend)
# ===========================================================================

class PPOAgent:
    """
    Proximal Policy Optimization agent.

    Parameters
    ----------
    obs_dim        : observation dimension
    act_dim        : action dimension (3 for Δkp, Δki, Δkd)
    lr             : learning rate
    gamma          : discount factor
    gae_lambda     : GAE λ
    clip_eps       : PPO clip epsilon
    entropy_coef   : entropy bonus coefficient
    value_coef     : value loss coefficient
    n_epochs       : optimisation epochs per rollout
    batch_size     : mini-batch size
    rollout_steps  : steps per rollout (before update)
    """

    def __init__(
        self,
        obs_dim:      int   = 9,
        act_dim:      int   = 3,
        lr:           float = 3e-4,
        gamma:        float = 0.99,
        gae_lambda:   float = 0.95,
        clip_eps:     float = 0.2,
        entropy_coef: float = 0.01,
        value_coef:   float = 0.5,
        n_epochs:     int   = 10,
        batch_size:   int   = 64,
        rollout_steps: int  = 512,
    ):
        if not TORCH_AVAILABLE:
            raise ImportError("PyTorch is required for PPOAgent. "
                              "Install with: pip install torch")

        self.gamma        = gamma
        self.gae_lambda   = gae_lambda
        self.clip_eps     = clip_eps
        self.entropy_coef = entropy_coef
        self.value_coef   = value_coef
        self.n_epochs     = n_epochs
        self.batch_size   = batch_size
        self.rollout_steps = rollout_steps

        self.policy = _ActorCritic(obs_dim, act_dim)
        self.optim  = optim.Adam(self.policy.parameters(), lr=lr)

        # Rollout buffer
        self._buf_obs    = []
        self._buf_acts   = []
        self._buf_logps  = []
        self._buf_rews   = []
        self._buf_dones  = []
        self._buf_vals   = []

        self._total_steps = 0
        self._training_log: List[Dict] = []

    # ------------------------------------------------------------------
    # Rollout & update
    # ------------------------------------------------------------------

    def collect_rollout(self, env: PIDTuningEnv) -> float:
        """
        Collect `rollout_steps` transitions.
        Returns mean episode reward.
        """
        self._buf_obs.clear();  self._buf_acts.clear()
        self._buf_logps.clear(); self._buf_rews.clear()
        self._buf_dones.clear(); self._buf_vals.clear()

        obs, _ = env.reset()
        ep_rewards = []
        ep_r = 0.0

        for _ in range(self.rollout_steps):
            obs_t = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                dist  = self.policy.get_dist(obs_t)
                act_t = dist.sample()
                logp  = dist.log_prob(act_t).sum(-1)
                _, _, val = self.policy(obs_t)

            action = act_t.squeeze(0).numpy()
            next_obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            self._buf_obs.append(obs)
            self._buf_acts.append(action)
            self._buf_logps.append(float(logp.item()))
            self._buf_rews.append(reward)
            self._buf_dones.append(float(done))
            self._buf_vals.append(float(val.item()))

            ep_r += reward
            obs = next_obs
            self._total_steps += 1

            if done:
                ep_rewards.append(ep_r)
                ep_r = 0.0
                obs, _ = env.reset()

        # Bootstrap last value
        obs_t = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            _, _, last_val = self.policy(obs_t)
        last_val = float(last_val.item())

        return np.mean(ep_rewards) if ep_rewards else 0.0

    def update(self) -> Dict[str, float]:
        """Run PPO update on the collected rollout buffer."""
        obs   = np.array(self._buf_obs,   dtype=np.float32)
        acts  = np.array(self._buf_acts,  dtype=np.float32)
        logps = np.array(self._buf_logps, dtype=np.float32)
        rews  = np.array(self._buf_rews,  dtype=np.float32)
        dones = np.array(self._buf_dones, dtype=np.float32)
        vals  = np.array(self._buf_vals,  dtype=np.float32)

        # GAE advantages
        advs  = np.zeros_like(rews)
        gae   = 0.0
        for i in reversed(range(len(rews))):
            next_val = vals[i+1] if i+1 < len(vals) else 0.0
            delta = rews[i] + self.gamma * next_val * (1-dones[i]) - vals[i]
            gae   = delta + self.gamma * self.gae_lambda * (1-dones[i]) * gae
            advs[i] = gae
        returns = advs + vals
        advs    = (advs - advs.mean()) / (advs.std() + 1e-8)

        obs_t    = torch.tensor(obs,     dtype=torch.float32)
        acts_t   = torch.tensor(acts,    dtype=torch.float32)
        logps_t  = torch.tensor(logps,   dtype=torch.float32)
        advs_t   = torch.tensor(advs,    dtype=torch.float32)
        returns_t = torch.tensor(returns, dtype=torch.float32)

        total_loss_p = total_loss_v = total_loss_e = 0.0
        n_updates = 0

        for _ in range(self.n_epochs):
            idx = np.random.permutation(len(obs))
            for start in range(0, len(obs), self.batch_size):
                mb = idx[start:start+self.batch_size]
                mb_obs  = obs_t[mb];  mb_acts = acts_t[mb]
                mb_old  = logps_t[mb]; mb_adv = advs_t[mb]
                mb_ret  = returns_t[mb]

                dist = self.policy.get_dist(mb_obs)
                new_logp = dist.log_prob(mb_acts).sum(-1)
                entropy  = dist.entropy().sum(-1).mean()
                _, _, new_val = self.policy(mb_obs)

                ratio  = (new_logp - mb_old).exp()
                surr1  = ratio * mb_adv
                surr2  = ratio.clamp(1-self.clip_eps, 1+self.clip_eps) * mb_adv
                loss_p = -torch.min(surr1, surr2).mean()
                loss_v =  ((new_val - mb_ret) ** 2).mean()
                loss   = loss_p + self.value_coef*loss_v - self.entropy_coef*entropy

                self.optim.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(), 0.5)
                self.optim.step()

                total_loss_p += float(loss_p.item())
                total_loss_v += float(loss_v.item())
                total_loss_e += float(entropy.item())
                n_updates    += 1

        return {
            "policy_loss": total_loss_p / max(n_updates, 1),
            "value_loss":  total_loss_v / max(n_updates, 1),
            "entropy":     total_loss_e / max(n_updates, 1),
        }

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def act(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        """Get action from current policy."""
        obs_t = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            if deterministic:
                mean, _, _ = self.policy(obs_t)
                return mean.squeeze(0).numpy()
            else:
                dist = self.policy.get_dist(obs_t)
                return dist.sample().squeeze(0).numpy()

    def suggest_gains(self, obs: np.ndarray) -> Dict[str, float]:
        """
        Run one deterministic inference step and return the suggested
        ΔKp, ΔKi, ΔKd as a dict with actual (not incremental) values.
        Note: call this from a running env to get the final gains.
        """
        action = self.act(obs, deterministic=True)
        return {"action_delta_kp": float(action[0]),
                "action_delta_ki": float(action[1]),
                "action_delta_kd": float(action[2])}

    # ------------------------------------------------------------------
    # Checkpoint
    # ------------------------------------------------------------------

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        torch.save(self.policy.state_dict(), path)
        print(f"[PPOAgent] Saved checkpoint → {path}")

    def load(self, path: str) -> None:
        self.policy.load_state_dict(torch.load(path, map_location="cpu"))
        self.policy.eval()
        print(f"[PPOAgent] Loaded checkpoint ← {path}")

    @property
    def training_log(self) -> List[Dict]:
        return self._training_log


# ===========================================================================
# Numpy-only fallback: simple random-search / ES agent
# ===========================================================================

class ESAgent:
    """
    Simple Evolution Strategy (CMA-ES inspired) gain tuner.
    Works without PyTorch.  Suitable for low-dimensional tuning (≤ 3 gains).

    Parameters
    ----------
    axis          : "pitch" | "roll" | "yaw"
    population    : ES population size
    sigma         : initial search std
    lr            : learning rate for mean update
    """

    def __init__(
        self,
        axis:       str   = "pitch",
        population: int   = 20,
        sigma:      float = 0.15,
        lr:         float = 0.1,
    ):
        self.axis   = axis
        self.pop    = population
        self.sigma  = sigma
        self.lr     = lr
        gr = _GAIN_RANGES[axis]
        # Mean gains (normalised 0-1)
        self._mu = np.array([0.3, 0.1, 0.05])
        self._best: Optional[Dict[str, float]] = None
        self._best_score = -np.inf

    def train_iteration(self, env: PIDTuningEnv) -> float:
        """One ES generation. Returns mean score."""
        noise    = np.random.randn(self.pop, 3)
        members  = np.clip(self._mu + self.sigma * noise, 0.0, 1.0)
        scores   = np.zeros(self.pop)

        for i, gains_n in enumerate(members):
            env.reset()
            gr = _GAIN_RANGES[self.axis]
            kp = gains_n[0] * (gr["kp"][1] - gr["kp"][0]) + gr["kp"][0]
            ki = gains_n[1] * (gr["ki"][1] - gr["ki"][0]) + gr["ki"][0]
            kd = gains_n[2] * (gr["kd"][1] - gr["kd"][0]) + gr["kd"][0]
            env._kp, env._ki, env._kd = kp, ki, kd
            env._pid.kp, env._pid.ki, env._pid.kd = kp, ki, kd

            total_r = 0.0
            obs, _ = env.reset()
            env._kp, env._ki, env._kd = kp, ki, kd
            env._pid.kp, env._pid.ki, env._pid.kd = kp, ki, kd
            for _ in range(env.episode_steps):
                _, r, term, trunc, _ = env.step(np.zeros(3))  # no RL, fixed gains
                total_r += r
                if term or trunc:
                    break
            scores[i] = total_r

            if total_r > self._best_score:
                self._best_score = total_r
                self._best = {"kp": kp, "ki": ki, "kd": kd}

        # Update mean towards high-scoring members
        elite_idx = np.argsort(scores)[-self.pop//2:]
        elite = members[elite_idx]
        grad  = np.mean((elite - self._mu) / self.sigma * (scores[elite_idx] - scores.mean())[:,None], axis=0)
        self._mu = np.clip(self._mu + self.lr * grad, 0.0, 1.0)

        return float(scores.mean())

    @property
    def best_gains(self) -> Optional[Dict[str, float]]:
        return self._best


# ===========================================================================
# Training entry point
# ===========================================================================

def train(
    axis:         str   = "pitch",
    total_steps:  int   = 50_000,
    save_path:    str   = None,
    verbose:      bool  = True,
    rollout_steps: int  = 512,
) -> PPOAgent:
    """
    Train a PPO agent to tune PID gains for the given axis.

    Parameters
    ----------
    axis          : "pitch" | "roll" | "yaw"
    total_steps   : total environment steps
    save_path     : if provided, save checkpoint here
    verbose       : print training progress
    rollout_steps : steps per PPO rollout

    Returns
    -------
    Trained PPOAgent
    """
    if not TORCH_AVAILABLE:
        raise ImportError("PyTorch required for PPO training. "
                          "Install with: pip install torch  OR  use ESAgent.")

    env   = PIDTuningEnv(axis=axis, episode_steps=500)
    agent = PPOAgent(rollout_steps=rollout_steps)
    n_updates = total_steps // rollout_steps
    t0 = time.time()

    for i in range(n_updates):
        mean_rew = agent.collect_rollout(env)
        metrics  = agent.update()
        agent._training_log.append({
            "update": i, "steps": agent._total_steps,
            "mean_reward": mean_rew, **metrics,
        })
        if verbose and (i % max(1, n_updates//20) == 0):
            elapsed = time.time() - t0
            print(f"  [{axis}] Update {i:4d}/{n_updates}  "
                  f"steps={agent._total_steps:7d}  "
                  f"rew={mean_rew:7.2f}  "
                  f"pl={metrics['policy_loss']:.4f}  "
                  f"vl={metrics['value_loss']:.4f}  "
                  f"t={elapsed:.0f}s")

    if save_path:
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        agent.save(save_path)

    return agent


# ===========================================================================
# Load checkpoint and push optimal gains to ParamStore
# ===========================================================================

def load_gains_from_checkpoint(
    checkpoint_path: str,
    store,           # ParamStore instance
    axis: str = "pitch",
    n_eval_steps: int = 500,
) -> Dict[str, float]:
    """
    Load a trained checkpoint, run one deterministic episode, and push
    the final learned gains to the given ParamStore.

    Returns
    -------
    Dict of ArduPilot parameter names → gain values
    """
    agent = PPOAgent()
    agent.load(checkpoint_path)
    agent.policy.eval()

    env = PIDTuningEnv(axis=axis, episode_steps=n_eval_steps)
    obs, _ = env.reset()
    for _ in range(n_eval_steps):
        action = agent.act(obs, deterministic=True)
        obs, _, term, trunc, info = env.step(action)
        if term or trunc:
            break

    gains = env.current_gains   # ArduPilot-named gains
    store.set_batch(gains)
    print(f"[RL] Loaded gains from {checkpoint_path}: {gains}")
    return gains
