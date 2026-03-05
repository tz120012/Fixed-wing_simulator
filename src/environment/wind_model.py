"""
wind_model.py  –  High-fidelity wind field for fixed-wing simulation.

Supported types
---------------
NONE        – zero wind
FIXED       – constant mean wind vector (speed + met direction)
DRYDEN      – MIL-HDBK-1797B / MIL-F-8785C Dryden turbulence model
              Discrete-time state-space shaping filters driven by white noise.
              Turbulence intensity and length scales vary with altitude.
GUST        – 1-cos deterministic gust envelope (independent per axis)
COMBINED    – FIXED mean + DRYDEN turbulence + optional GUST (superposition)

NED convention throughout:  +North, +East, +Down
"""

import numpy as np
from typing import Optional


# ---------------------------------------------------------------------------
# Helper: standard atmosphere for turbulence scaling
# ---------------------------------------------------------------------------

def _isa_density(alt_m: float) -> float:
    """Air density via simple ISA (kg/m³), alt in m above MSL."""
    T = max(288.15 - 0.0065 * alt_m, 216.65)
    p_ratio = (T / 288.15) ** 5.2561 if alt_m <= 11000 else \
              0.22336 * np.exp(-9.80665 * (alt_m - 11000) / (287.05 * 216.65))
    return 1.225 * (T / 288.15) ** 4.2561 if alt_m <= 11000 else \
           1.225 * 0.22336 * (216.65 / 288.15) ** 4.2561


# ---------------------------------------------------------------------------
# Dryden turbulence intensity and length scales  (MIL-HDBK-1797B, Table C-1)
# ---------------------------------------------------------------------------

def _dryden_params(alt_m: float, severity: str = "moderate") -> dict:
    """
    Return σ (m/s) and L (m) for u/v/w axes at the given altitude.

    severity : 'light' | 'moderate' | 'severe'
    Altitude regions:
      low  : alt ≤  300 m  (near ground, MIL-F-8785C Table 3-2)
      mid  : 300 < alt ≤ 1800 m
      high : alt > 1800 m  (high-altitude standard atmosphere)
    """
    h = max(alt_m, 1.0)          # prevent log(0)

    # ---- turbulence intensities (m/s) ----
    scale = {"light": 0.5, "moderate": 1.0, "severe": 2.0}[severity]

    if h <= 300.0:
        # Near-ground: MIL-F-8785C low-altitude model
        # σ_w scales with h^(1/6), σ_u = σ_v = σ_w (simplified)
        sigma_w = scale * 0.1 * (h / 300.0) ** (1.0 / 6.0) * 15.0
        sigma_w = np.clip(sigma_w, 0.5, 15.0)
        sigma_u = sigma_v = sigma_w
        # Length scales (m)
        L_w = h
        L_u = L_v = h / (0.177 + 0.000823 * h) ** 1.2
    elif h <= 1800.0:
        # Mid altitude: linear interpolation
        frac = (h - 300.0) / 1500.0
        sigma_w = scale * (1.5 + frac * 1.5)
        sigma_u = sigma_v = sigma_w
        L_w = 300.0 + frac * 1500.0
        L_u = L_v = L_w
    else:
        # High altitude
        sigma_w = scale * 3.0
        sigma_u = sigma_v = sigma_w
        L_u = L_v = L_w = 1750.0

    return dict(sigma_u=sigma_u, sigma_v=sigma_v, sigma_w=sigma_w,
                L_u=L_u, L_v=L_v, L_w=L_w)


# ---------------------------------------------------------------------------
# Dryden shaping filter (discrete-time state-space, per axis)
# ---------------------------------------------------------------------------

class _DrydenFilter:
    """
    Discrete-time Dryden shaping filter (MIL-HDBK-1797B / MIL-F-8785C).

    Continuous-time transfer functions (input = unit-PSD white noise w(t)):

      Longitudinal (u-axis, order-1):
          H_u(s) = σ_u * sqrt(2*L_u / (π*V))  /  (1 + τ_u*s)
          τ_u = L_u / V

      Lateral / vertical (v,w-axes, order-2):
          H_v(s) = σ_v * sqrt(L_v / (π*V))  *  (1 + √3·τ_v·s)
                   / (1 + τ_v·s)²
          τ_v = L_v / V

    Discretisation: Euler forward at step dt.
    Input white noise has PSD = 1  →  per-step std = 1/√dt.

    State-space (order-2):
        ẋ₁ =  x₂
        ẋ₂ = -(2/τ)·x₂  -  (1/τ²)·x₁  +  K_v · w(t)
        y   =  x₁  +  √3·τ·x₂

    where  K_v = σ_v * sqrt(L_v / (π*V)) * (√3/τ²)
    and the noise input std per step = 1/√dt  (unit-PSD discretisation).

    State-space (order-1):
        ẋ = -(1/τ)·x  +  K_u · w(t)
        y = x

    where  K_u = σ_u * sqrt(2*L_u / (π*V)) / τ
    """

    def __init__(self, sigma: float, L: float, V: float, dt: float,
                 order: int = 1, rng: Optional[np.random.Generator] = None):
        """
        Parameters
        ----------
        sigma  : turbulence intensity (m/s)
        L      : length scale (m)
        V      : airspeed (m/s) – frozen-field conversion
        dt     : simulation time step (s)
        order  : 1 for u-axis, 2 for v/w-axes
        rng    : numpy random Generator
        """
        self.sigma = sigma
        self.L     = L
        self.V     = max(V, 1.0)
        self.dt    = dt
        self.order = order
        self.rng   = rng or np.random.default_rng()
        self.x     = np.zeros(order)

        # Input white noise: unit PSD → per-step std = 1/sqrt(dt)
        # (This is constant; filter gains carry all σ scaling.)
        self._noise_std = 1.0 / np.sqrt(dt)

    def update(self, V: Optional[float] = None) -> float:
        """Advance one dt step. Returns turbulence component (m/s)."""
        if V is not None:
            self.V = max(V, 1.0)

        V   = self.V
        L   = self.L
        dt  = self.dt
        tau = L / V                   # time constant (s)

        # unit-PSD white noise sample
        w = self.rng.standard_normal() * self._noise_std

        if self.order == 1:
            # Continuous:  ẋ = -(1/τ)x + K_u·w
            # K_u = σ·sqrt(2L/πV) / τ
            # Euler discrete:  x_{k+1} = (1 - dt/τ)·x_k + dt·K_u·w_k
            K_u = self.sigma * np.sqrt(2.0 * L / (np.pi * V)) / tau
            a   = 1.0 - dt / tau
            b   = dt * K_u
            self.x[0] = a * self.x[0] + b * w
            return float(self.x[0])

        else:
            # Continuous:  ẋ₁ = x₂
            #              ẋ₂ = -(2/τ)x₂ - (1/τ²)x₁ + K_v·w
            # K_v = σ·sqrt(L/πV)·√3/τ²
            # Output:  y = x₁ + √3·τ·x₂
            K_v = self.sigma * np.sqrt(L / (np.pi * V)) * np.sqrt(3.0) / (tau ** 2)
            a11 = 1.0
            a12 = dt
            a21 = -dt / (tau ** 2)
            a22 = 1.0 - 2.0 * dt / tau
            b2  = dt * K_v
            x1_new = a11 * self.x[0] + a12 * self.x[1]
            x2_new = a21 * self.x[0] + a22 * self.x[1] + b2 * w
            self.x[0] = x1_new
            self.x[1] = x2_new
            return float(self.x[0] + np.sqrt(3.0) * tau * self.x[1])


# ---------------------------------------------------------------------------
# Gust envelope  (1-cosine deterministic gust, per axis)
# ---------------------------------------------------------------------------

class _GustEnvelope:
    """
    Deterministic 1-cosine gust envelope (MIL-HDBK-1797B §2.4).

    Each gust is defined by:
      - peak amplitude  U_ds  (m/s)
      - gradient distance d_m (m)  → duration = 2*d_m / V
      - trigger time  t_start (s)

    The gust shape: u_g(t) = U_ds/2 * (1 - cos(π*(t-t0)/d_m*V))
                             for t0 ≤ t ≤ t0 + 2*d_m/V
    """

    def __init__(
        self,
        gusts: list,          # list of dicts: {axis, amplitude, gradient_m, t_start}
        V_ref: float = 40.0,  # reference airspeed for duration calc (m/s)
    ):
        """
        Parameters
        ----------
        gusts : list of gust definitions, each dict with keys:
            axis       : 0=North, 1=East, 2=Down (NED)
            amplitude  : peak gust speed (m/s), positive or negative
            gradient_m : gust gradient distance d_m (m), typ. 100–350 m
            t_start    : gust start time (s)
        V_ref : reference airspeed for computing gust duration (m/s)
        """
        self._gusts = gusts
        self._V_ref = V_ref

    def get_gust_ned(self, t: float, V: Optional[float] = None) -> np.ndarray:
        """Return NED gust vector at time t."""
        V = V or self._V_ref
        w = np.zeros(3)
        for g in self._gusts:
            t0  = g["t_start"]
            d_m = g["gradient_m"]
            U   = g["amplitude"]
            ax  = g["axis"]
            dur = 2.0 * d_m / max(V, 1.0)
            if t0 <= t <= t0 + dur:
                phase = np.pi * (t - t0) / (d_m / max(V, 1.0))
                w[ax] += 0.5 * U * (1.0 - np.cos(phase))
        return w


# ---------------------------------------------------------------------------
# Public Wind class
# ---------------------------------------------------------------------------

class Wind:
    """
    High-fidelity wind field generator.

    Parameters
    ----------
    wind_type : str
        'NONE'     – zero wind
        'FIXED'    – constant mean wind
        'DRYDEN'   – Dryden turbulence only (no mean wind)
        'GUST'     – 1-cosine deterministic gust(s) only
        'COMBINED' – FIXED mean + DRYDEN turbulence + optional GUST

    speed         : mean wind speed (m/s), used by FIXED / COMBINED
    direction_deg : wind FROM direction (met convention, deg;
                    0=from North, 90=from East)
    altitude_m    : representative cruise altitude (m) for turbulence
                    scaling; can be updated at runtime via set_altitude()
    airspeed_mps  : representative airspeed (m/s) for frozen-field conversion
    severity      : Dryden turbulence intensity 'light'|'moderate'|'severe'
    dt            : simulation time step (s)
    gusts         : list of gust dicts for GUST/COMBINED modes
    seed          : RNG seed for reproducibility
    """

    TYPES = ("NONE", "FIXED", "DRYDEN", "GUST", "COMBINED",
             # Legacy aliases kept for backward compatibility
             "SINE", "RANDOMSINE")

    def __init__(
        self,
        wind_type:      str   = "NONE",
        speed:          float = 5.0,
        direction_deg:  float = 270.0,
        altitude_m:     float = 100.0,
        airspeed_mps:   float = 40.0,
        severity:       str   = "moderate",
        dt:             float = 0.01,
        gusts:          Optional[list] = None,
        seed:           int   = 42,
    ):
        wt = wind_type.upper()
        if wt not in self.TYPES:
            raise ValueError(f"Unknown wind type '{wind_type}'. Choose from {self.TYPES}")

        self.wind_type     = wt
        self.speed         = float(speed)
        self.direction_deg = float(direction_deg)
        self._altitude     = float(altitude_m)
        self._airspeed     = float(airspeed_mps)
        self._severity     = severity
        self._dt           = dt

        self._rng = np.random.default_rng(seed)

        # ── Mean wind vector (NED) ──────────────────────────────────────────
        # "Wind FROM direction_deg" blows TOWARD (direction_deg + 180)
        heading_rad = np.deg2rad(direction_deg + 180.0)
        self._fixed_ned = self.speed * np.array([
            np.cos(heading_rad),
            np.sin(heading_rad),
            0.0,
        ])

        # ── Dryden filters ──────────────────────────────────────────────────
        self._dryden_u: Optional[_DrydenFilter] = None
        self._dryden_v: Optional[_DrydenFilter] = None
        self._dryden_w: Optional[_DrydenFilter] = None

        if wt in ("DRYDEN", "COMBINED"):
            self._init_dryden_filters(altitude_m, airspeed_mps, dt)

        # ── Gust envelope ───────────────────────────────────────────────────
        self._gust_env: Optional[_GustEnvelope] = None
        if wt in ("GUST", "COMBINED") and gusts:
            self._gust_env = _GustEnvelope(gusts, V_ref=airspeed_mps)

        # ── Legacy SINE / RANDOMSINE (backward compatibility) ───────────────
        if wt in ("SINE", "RANDOMSINE"):
            n_sin = 3
            self._freqs  = self._rng.uniform(0.1, 0.5, (3, n_sin))
            self._phases = self._rng.uniform(0, 2 * np.pi, (3, n_sin))
            if wt == "SINE":
                self._amps = np.full((3, n_sin), self.speed / n_sin)
            else:
                self._amps  = self._rng.uniform(0, self.speed, (3, n_sin))
                self._means = self._rng.uniform(
                    -self.speed * 0.5, self.speed * 0.5, 3)

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _init_dryden_filters(self, alt: float, V: float, dt: float) -> None:
        """(Re-)initialise Dryden shaping filters for given altitude / speed."""
        p = _dryden_params(alt, self._severity)
        rng = self._rng
        self._dryden_u = _DrydenFilter(p["sigma_u"], p["L_u"], V, dt,
                                       order=1, rng=rng)
        self._dryden_v = _DrydenFilter(p["sigma_v"], p["L_v"], V, dt,
                                       order=2, rng=rng)
        self._dryden_w = _DrydenFilter(p["sigma_w"], p["L_w"], V, dt,
                                       order=2, rng=rng)

    # -----------------------------------------------------------------------
    # Runtime parameter updates
    # -----------------------------------------------------------------------

    def set_altitude(self, alt_m: float) -> None:
        """Update turbulence parameters for a new cruise altitude."""
        if self.wind_type in ("DRYDEN", "COMBINED"):
            self._altitude = float(alt_m)
            self._init_dryden_filters(alt_m, self._airspeed, self._dt)

    def set_airspeed(self, V_mps: float) -> None:
        """Update frozen-field conversion speed (call when airspeed changes)."""
        self._airspeed = max(float(V_mps), 1.0)
        if self._dryden_u is not None:
            self._dryden_u.V = self._airspeed
        if self._dryden_v is not None:
            self._dryden_v.V = self._airspeed
        if self._dryden_w is not None:
            self._dryden_w.V = self._airspeed

    # -----------------------------------------------------------------------
    # Public interface
    # -----------------------------------------------------------------------

    def get_wind_ned(self, t: float,
                     V: Optional[float] = None,
                     alt: Optional[float] = None) -> np.ndarray:
        """
        Return the NED wind vector at simulation time *t*.

        Parameters
        ----------
        t   : simulation time (s)
        V   : current airspeed (m/s), optional – updates frozen-field scale
        alt : current altitude (m), optional – updates turbulence intensity

        Returns
        -------
        (3,) array  [v_north, v_east, v_down]  in m/s
        """
        if V is not None:
            self.set_airspeed(V)
        if alt is not None and alt != self._altitude:
            self.set_altitude(alt)

        wt = self.wind_type

        # ── NONE ────────────────────────────────────────────────────────────
        if wt == "NONE":
            return np.zeros(3)

        # ── FIXED ───────────────────────────────────────────────────────────
        if wt == "FIXED":
            return self._fixed_ned.copy()

        # ── DRYDEN turbulence only ──────────────────────────────────────────
        if wt == "DRYDEN":
            return self._sample_dryden()

        # ── GUST only ──────────────────────────────────────────────────────
        if wt == "GUST":
            return self._gust_env.get_gust_ned(t, self._airspeed) \
                   if self._gust_env else np.zeros(3)

        # ── COMBINED  (mean + turbulence + gust) ───────────────────────────
        if wt == "COMBINED":
            w = self._fixed_ned.copy()
            w += self._sample_dryden()
            if self._gust_env:
                w += self._gust_env.get_gust_ned(t, self._airspeed)
            return w

        # ── Legacy SINE / RANDOMSINE ────────────────────────────────────────
        if wt == "SINE":
            w = np.zeros(3)
            for ax in range(3):
                for k in range(self._freqs.shape[1]):
                    w[ax] += self._amps[ax, k] * np.sin(
                        2 * np.pi * self._freqs[ax, k] * t + self._phases[ax, k])
            return w

        if wt == "RANDOMSINE":
            w = self._means.copy()
            for ax in range(3):
                for k in range(self._freqs.shape[1]):
                    w[ax] += self._amps[ax, k] * np.sin(
                        2 * np.pi * self._freqs[ax, k] * t + self._phases[ax, k])
            return w

        return np.zeros(3)

    # -----------------------------------------------------------------------
    # Private helpers
    # -----------------------------------------------------------------------

    def _sample_dryden(self) -> np.ndarray:
        """Step all three Dryden filters and return NED turbulence vector."""
        if self._dryden_u is None:
            return np.zeros(3)
        # u-axis → maps to North (along-track, approx), v → East, w → Down
        # For generality we assign: [0]=North (u), [1]=East (v), [2]=Down (w)
        tu = self._dryden_u.update()
        tv = self._dryden_v.update()
        tw = self._dryden_w.update()
        return np.array([tu, tv, tw])

    def __repr__(self) -> str:
        return (f"Wind(type={self.wind_type}, speed={self.speed:.1f} m/s, "
                f"dir={self.direction_deg:.0f}°, severity={self._severity})")
