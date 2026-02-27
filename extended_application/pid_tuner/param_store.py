"""
param_store.py  –  Thread-safe PID parameter store.

Central registry that decouples GUI / RL agent from the simulation loop.
Any thread can write new gains; the simulation loop polls `has_update()`
and calls `apply_to_sim(sim)` to hot-reload controllers without restart.

Architecture
------------
  GUI thread  ─┐
  RL  thread  ─┼──▶  ParamStore (RLock)  ──▶  FixedWingSimulator.reload_gains()
  Script API  ─┘

Supported parameter groups (mirrors ArdupilotParams field names):
  Pitch  : PTCH_P, PTCH_D, PTCH_RATE_P, PTCH_RATE_I, PTCH_RATE_D, PTCH_RATE_FF
  Roll   : ROLL_P, ROLL_D, ROLL_RATE_P, ROLL_RATE_I, ROLL_RATE_FF
  Yaw    : YAW_P, YAW_RATE_P, YAW_RATE_I
  Nav    : NAVL1_PERIOD, NAVL1_DAMPING
  Speed  : AIRSPEED_CRUISE
"""

from __future__ import annotations

import copy
import json
import os
import threading
import time
from dataclasses import dataclass, asdict
from typing import Any, Callable, Dict, List, Optional, Tuple

import yaml


# ---------------------------------------------------------------------------
# Default PID parameter set (conservative, matches ArdupilotParams defaults)
# ---------------------------------------------------------------------------

DEFAULT_PARAMS: Dict[str, float] = {
    # Pitch attitude outer loop
    "PTCH_P":       1.0,
    "PTCH_D":       0.08,
    # Pitch rate inner loop
    "PTCH_RATE_P":  0.04,
    "PTCH_RATE_I":  0.10,
    "PTCH_RATE_D":  0.002,
    "PTCH_RATE_FF": 0.15,
    # Roll attitude outer loop
    "ROLL_P":       1.0,
    "ROLL_D":       0.05,
    # Roll rate inner loop
    "ROLL_RATE_P":  0.05,
    "ROLL_RATE_I":  0.10,
    "ROLL_RATE_FF": 0.15,
    # Yaw
    "YAW_P":        0.5,
    "YAW_RATE_P":   0.02,
    "YAW_RATE_I":   0.0,
    # Navigation
    "NAVL1_PERIOD":  25.0,
    "NAVL1_DAMPING": 0.75,
    # Speed
    "AIRSPEED_CRUISE": 30.0,
}

# Allowed ranges for each parameter [lo, hi]
PARAM_RANGES: Dict[str, Tuple[float, float]] = {
    "PTCH_P":        (0.0,  10.0),
    "PTCH_D":        (0.0,   1.0),
    "PTCH_RATE_P":   (0.0,   2.0),
    "PTCH_RATE_I":   (0.0,   2.0),
    "PTCH_RATE_D":   (0.0,   0.1),
    "PTCH_RATE_FF":  (0.0,   1.0),
    "ROLL_P":        (0.0,  10.0),
    "ROLL_D":        (0.0,   1.0),
    "ROLL_RATE_P":   (0.0,   2.0),
    "ROLL_RATE_I":   (0.0,   2.0),
    "ROLL_RATE_FF":  (0.0,   1.0),
    "YAW_P":         (0.0,   5.0),
    "YAW_RATE_P":    (0.0,   1.0),
    "YAW_RATE_I":    (0.0,   1.0),
    "NAVL1_PERIOD":  (5.0,  60.0),
    "NAVL1_DAMPING": (0.1,   2.0),
    "AIRSPEED_CRUISE": (5.0, 100.0),
}

# Human-readable group layout (used by GUI)
PARAM_GROUPS: Dict[str, List[str]] = {
    "Pitch Attitude": ["PTCH_P", "PTCH_D"],
    "Pitch Rate":     ["PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF"],
    "Roll Attitude":  ["ROLL_P", "ROLL_D"],
    "Roll Rate":      ["ROLL_RATE_P", "ROLL_RATE_I", "ROLL_RATE_FF"],
    "Yaw":            ["YAW_P", "YAW_RATE_P", "YAW_RATE_I"],
    "Navigation":     ["NAVL1_PERIOD", "NAVL1_DAMPING"],
    "Speed":          ["AIRSPEED_CRUISE"],
}


# ---------------------------------------------------------------------------
# ParamStore
# ---------------------------------------------------------------------------

class ParamStore:
    """
    Thread-safe parameter registry with change notification.

    Usage
    -----
    store = ParamStore()

    # Register a callback (called on any parameter change)
    store.register_callback(lambda params: print("Changed:", params))

    # Update a single gain
    store.set("PTCH_P", 1.5)

    # Update multiple gains at once
    store.set_batch({"PTCH_RATE_P": 0.06, "PTCH_RATE_I": 0.12})

    # Check whether the simulation loop should reload
    if store.has_update():
        store.apply_to_sim(simulator)

    # Persist to YAML
    store.save_yaml("config/control_params.yaml")
    """

    def __init__(self, initial: Optional[Dict[str, float]] = None):
        self._lock    = threading.RLock()
        self._params: Dict[str, float] = dict(DEFAULT_PARAMS)
        if initial:
            self.set_batch(initial, _notify=False)

        self._dirty   = False          # True if params changed since last apply
        self._version = 0              # monotonic counter
        self._callbacks: List[Callable[[Dict[str, float]], None]] = []

        # History of parameter snapshots for RL observation (ring buffer)
        self._history_maxlen = 200
        self._history: List[Dict[str, Any]] = []

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def get(self, name: str) -> float:
        with self._lock:
            if name not in self._params:
                raise KeyError(f"Unknown parameter: {name}")
            return self._params[name]

    def get_all(self) -> Dict[str, float]:
        """Return a shallow copy of the full parameter dict."""
        with self._lock:
            return dict(self._params)

    @property
    def version(self) -> int:
        """Monotonic change counter. RL env can use this to detect updates."""
        with self._lock:
            return self._version

    def has_update(self) -> bool:
        """True if params changed since the last call to clear_update()."""
        with self._lock:
            return self._dirty

    def clear_update(self) -> None:
        with self._lock:
            self._dirty = False

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def set(self, name: str, value: float, clamp: bool = True) -> None:
        """Set a single parameter, optionally clamping to allowed range."""
        if name not in DEFAULT_PARAMS:
            raise KeyError(f"Unknown parameter '{name}'. Valid: {list(DEFAULT_PARAMS)}")
        if clamp and name in PARAM_RANGES:
            lo, hi = PARAM_RANGES[name]
            value = max(lo, min(hi, float(value)))
        else:
            value = float(value)
        with self._lock:
            self._params[name] = value
            self._dirty   = True
            self._version += 1
        self._notify()

    def set_batch(
        self,
        updates: Dict[str, float],
        clamp: bool = True,
        _notify: bool = True,
    ) -> None:
        """Set multiple parameters atomically."""
        for name, value in updates.items():
            if name not in DEFAULT_PARAMS:
                raise KeyError(f"Unknown parameter '{name}'")
            if clamp and name in PARAM_RANGES:
                lo, hi = PARAM_RANGES[name]
                value = max(lo, min(hi, float(value)))
            updates[name] = float(value)
        with self._lock:
            self._params.update(updates)
            self._dirty   = True
            self._version += 1
        if _notify:
            self._notify()

    def reset_defaults(self) -> None:
        """Restore factory defaults."""
        with self._lock:
            self._params  = dict(DEFAULT_PARAMS)
            self._dirty   = True
            self._version += 1
        self._notify()

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def save_yaml(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with self._lock:
            data = dict(self._params)
        with open(path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=True)

    def load_yaml(self, path: str) -> None:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        self.set_batch({k: float(v) for k, v in data.items() if k in DEFAULT_PARAMS})

    def save_json(self, path: str) -> None:
        with self._lock:
            data = dict(self._params)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load_json(self, path: str) -> None:
        with open(path, "r") as f:
            data = json.load(f)
        self.set_batch({k: float(v) for k, v in data.items() if k in DEFAULT_PARAMS})

    # ------------------------------------------------------------------
    # Sim integration
    # ------------------------------------------------------------------

    def apply_to_sim(self, sim) -> None:
        """
        Hot-reload all parameters into a running FixedWingSimulator instance.
        Calls reload_gains() on AttitudeController, RateController, ServoMixer,
        and updates NavigationController scalars.
        """
        import sys, os
        _src = os.path.join(os.path.dirname(__file__), "..", "..", "src")
        if _src not in sys.path:
            sys.path.insert(0, _src)
        from control.ardupilot_compat import ArdupilotParams

        with self._lock:
            params = dict(self._params)
            self._dirty = False

        # Rebuild ArdupilotParams from store
        ap = ArdupilotParams.from_dict(params)
        sim.ap_params = ap

        # Hot-reload each controller layer
        sim.att_ctrl.reload_gains(ap)
        sim.rate_ctrl.reload_gains(ap)
        sim.servo.ap = ap

        # Navigation controller scalars
        sim.nav_ctrl.l1_period    = ap.NAVL1_PERIOD
        sim.nav_ctrl.l1_damping   = ap.NAVL1_DAMPING
        sim.nav_ctrl.cruise_speed = ap.AIRSPEED_CRUISE
        sim.nav_ctrl.k_speed      = 0.05   # keep fixed; tuned separately

        # Record snapshot to history
        self._record_snapshot(params)

    def _record_snapshot(self, params: Dict[str, float]) -> None:
        snap = {"timestamp": time.time(), **params}
        self._history.append(snap)
        if len(self._history) > self._history_maxlen:
            self._history = self._history[-self._history_maxlen:]

    def get_history(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._history)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def register_callback(self, fn: Callable[[Dict[str, float]], None]) -> None:
        """Register a function called (in the calling thread) on each update."""
        self._callbacks.append(fn)

    def _notify(self) -> None:
        snapshot = self.get_all()
        for fn in self._callbacks:
            try:
                fn(snapshot)
            except Exception as e:
                print(f"[ParamStore] callback error: {e}")

    # ------------------------------------------------------------------
    # Repr
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        with self._lock:
            v = self._version
        return f"ParamStore(version={v}, dirty={self._dirty})"
