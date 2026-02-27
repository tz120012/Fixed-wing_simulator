"""
config_loader.py  –  YAML configuration loading and merging utilities.
"""

import os
import yaml
from typing import Any, Dict


_DEFAULTS = {
    "aircraft": {
        "aircraft_name": "TB2",
        "overrides": {},
    },
    "simulation": {
        "dt": 0.01,
        "duration": 30.0,
        "integrator": "dopri5",
        "rtol": 1e-6,
        "atol": 1e-6,
        "initial_position": [0.0, 0.0, -100.0],
        "initial_heading_deg": 0.0,
        "initial_mode": "AUTO",
        "wind_type": "NONE",
        "wind_speed": 5.0,
        "wind_direction_deg": 270.0,
        "log_enabled": True,
        "log_dir": "logs",
    },
    "trajectory": {
        "type": "minimum_snap",
        "average_speed": 30.0,
        "yaw_mode": "yaw_follow",
        "waypoints": [[0, 0, 100]],
        "loop": False,
    },
}


def _load_yaml(path: str) -> Dict[str, Any]:
    if not os.path.isfile(path):
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base* (in-place on a copy)."""
    result = base.copy()
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


class ConfigLoader:
    """Load and merge YAML configuration files for all subsystems."""

    def __init__(self, config_dir: str = "config"):
        self.config_dir = config_dir

    def _path(self, filename: str) -> str:
        return os.path.join(self.config_dir, filename)

    def load_aircraft(self) -> Dict[str, Any]:
        data = _load_yaml(self._path("aircraft.yaml"))
        return _deep_merge(_DEFAULTS["aircraft"], data)

    def load_control(self) -> Dict[str, Any]:
        return _load_yaml(self._path("control_params.yaml"))

    def load_simulation(self) -> Dict[str, Any]:
        data = _load_yaml(self._path("simulation.yaml"))
        return _deep_merge(_DEFAULTS["simulation"], data)

    def load_trajectory(self) -> Dict[str, Any]:
        data = _load_yaml(self._path("trajectory.yaml"))
        return _deep_merge(_DEFAULTS["trajectory"], data)
