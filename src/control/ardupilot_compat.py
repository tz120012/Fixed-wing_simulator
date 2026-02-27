"""
ardupilot_compat.py  –  ArduPilot-compatible parameter container.

Provides ArdupilotParams dataclass whose field names exactly match the
ArduPilot Plane parameter naming convention. Supports load/save from YAML
and parameter validation.
"""

from __future__ import annotations

import os
import yaml
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Optional


@dataclass
class ArdupilotParams:
    """
    Standard ArduPilot Plane control parameters.

    Default values are conservative starting points suitable for a medium-
    sized fixed-wing UAV (e.g. TB2 class).
    """

    # ---- Pitch axis ----
    PTCH_P:       float = 1.0
    PTCH_D:       float = 0.08
    PTCH_RATE_P:  float = 0.04
    PTCH_RATE_I:  float = 0.10
    PTCH_RATE_D:  float = 0.002
    PTCH_RATE_FF: float = 0.15

    # ---- Roll axis ----
    ROLL_P:       float = 1.0
    ROLL_D:       float = 0.05
    ROLL_RATE_P:  float = 0.05
    ROLL_RATE_I:  float = 0.10
    ROLL_RATE_FF: float = 0.15

    # ---- Yaw axis ----
    YAW_P:        float = 0.5
    YAW_RATE_P:   float = 0.02
    YAW_RATE_I:   float = 0.0

    # ---- Limits ----
    LIM_PITCH_MAX:  float = 20.0   # deg
    LIM_PITCH_MIN:  float = -5.0   # deg
    LIM_ROLL_CD:    float = 4500.0 # centidegrees → 45 deg
    THR_MAX:        float = 1.0
    THR_MIN:        float = 0.0

    # ---- Navigation ----
    NAVL1_PERIOD:  float = 25.0
    NAVL1_DAMPING: float = 0.75

    # ---- Speed / altitude ----
    AIRSPEED_CRUISE: float = 30.0  # m/s
    ALT_HOLD_RTL:    float = 50.0  # m

    # ------------------------------------------------------------------
    # Convenience properties
    # ------------------------------------------------------------------

    @property
    def LIM_ROLL_DEG(self) -> float:
        """Maximum roll angle in degrees (converted from centidegrees)."""
        return self.LIM_ROLL_CD / 100.0

    # ------------------------------------------------------------------
    # Factory / serialisation
    # ------------------------------------------------------------------

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ArdupilotParams":
        """Create from a flat parameter dict (unknown keys are ignored)."""
        valid = {f.name for f in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        filtered = {k: float(v) for k, v in d.items() if k in valid}
        return cls(**filtered)

    @classmethod
    def from_yaml(cls, path: str) -> "ArdupilotParams":
        """Load from a YAML file (flat key:value format)."""
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Control params file not found: {path}")
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        return cls.from_dict(data)

    def to_dict(self) -> Dict[str, float]:
        """Export as flat dict."""
        return {k: float(v) for k, v in asdict(self).items()}

    def to_yaml(self, path: str) -> None:
        """Save parameters to a YAML file."""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False, sort_keys=True)

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def validate(self) -> bool:
        """
        Basic range checks.  Prints warnings for out-of-range values.

        Returns True if all checks pass, False otherwise.
        """
        ok = True
        checks = [
            ("PTCH_P",      0.0, 10.0),
            ("PTCH_RATE_P", 0.0,  2.0),
            ("PTCH_RATE_I", 0.0,  2.0),
            ("ROLL_P",      0.0, 10.0),
            ("ROLL_RATE_P", 0.0,  2.0),
            ("YAW_P",       0.0,  5.0),
            ("LIM_PITCH_MAX", 0.0, 45.0),
            ("LIM_PITCH_MIN", -20.0, 0.0),
            ("LIM_ROLL_CD",  0.0, 9000.0),
            ("THR_MAX",      0.0,  1.0),
            ("THR_MIN",      0.0,  1.0),
            ("AIRSPEED_CRUISE", 5.0, 200.0),
        ]
        for name, lo, hi in checks:
            val = getattr(self, name)
            if not (lo <= val <= hi):
                print(f"[ArdupilotParams] WARNING: {name}={val} outside [{lo}, {hi}]")
                ok = False
        return ok
