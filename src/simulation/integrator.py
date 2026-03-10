"""
integrator.py  –  Numerical integrator wrappers.

Two integrators are provided:
  Dopri5Integrator  – real-time step-by-step (scipy.integrate.ode, dopri5)
                      mirrors Quadcopter_SimCon's quad.py integrator
  RK45Integrator    – batch solve_ivp (for linear / offline analysis)
"""

from __future__ import annotations

import numpy as np
from typing import Callable, Optional
from scipy.integrate import ode, solve_ivp


class Dopri5Integrator:
    """
    Runge-Kutta 4(5) (Dormand-Prince) step-by-step integrator.

    Wraps scipy.integrate.ode with the dopri5 solver, which supports
    adaptive step-size control while still exposing a single-step API.

    Parameters
    ----------
    f     : callable(t, y) → dy/dt
    y0    : (n,) initial state
    t0    : initial time (s)
    rtol  : relative tolerance
    atol  : absolute tolerance
    """

    def __init__(
        self,
        f:    Callable,
        y0:   np.ndarray,
        t0:   float = 0.0,
        rtol: float = 1e-4,
        atol: float = 1e-6,
    ):
        self._r = ode(f).set_integrator(
            "dopri5",
            rtol=rtol,
            atol=atol,
            nsteps=3000,  # 增加步数上限，避免 RL 训练中极端动作导致积分失败
            verbosity=0,
        )
        self._r.set_initial_value(y0, t0)

    def step(self, dt: float) -> np.ndarray:
        """Advance by *dt* seconds; return new state vector."""
        t_next = self._r.t + dt
        self._r.integrate(t_next)
        if not self._r.successful():
            raise RuntimeError(f"Dopri5 integration failed at t={self._r.t:.4f}")
        return self._r.y.copy()

    @property
    def t(self) -> float:
        """Current integrator time (s)."""
        return float(self._r.t)

    @property
    def y(self) -> np.ndarray:
        """Current state vector."""
        return self._r.y.copy()

    def reset(self, y0: np.ndarray, t0: float = 0.0) -> None:
        """Re-initialise the integrator."""
        self._r.set_initial_value(y0, t0)


class RK45Integrator:
    """
    Batch integrator using scipy.integrate.solve_ivp (RK45).

    Suitable for offline linear/nonlinear analysis where the entire time
    history is needed at once.
    """

    def __init__(self, rtol: float = 1e-6, atol: float = 1e-6):
        self.rtol = rtol
        self.atol = atol

    def integrate(
        self,
        f:        Callable,
        y0:       np.ndarray,
        t_span:   tuple,
        t_eval:   Optional[np.ndarray] = None,
        max_step: float = 0.1,
    ):
        """
        Integrate ODE over *t_span*.

        Returns
        -------
        sol : scipy OdeResult  (sol.t, sol.y)
        """
        return solve_ivp(
            f, t_span, y0,
            t_eval=t_eval,
            rtol=self.rtol,
            atol=self.atol,
            max_step=max_step,
            method="RK45",
        )
