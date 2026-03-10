"""
aircraft_factory.py  –  Aircraft factory: load, merge and validate aircraft configs.
"""

from __future__ import annotations

import os
import yaml
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from models.aircraft_database import get_aircraft_params, AIRCRAFT_NAMES


@dataclass
class AircraftConfig:
    """
    Combined aircraft configuration consumed by the simulation engine.

    Attributes
    ----------
    name        : aircraft name (key in database)
    aero_params : merged aerodynamic/physical parameter dict
    """
    name:        str
    aero_params: Dict[str, Any] = field(default_factory=dict)

    def summary(self) -> str:
        p  = self.aero_params
        U0 = p.get("U0", 0.0)
        return (
            f"Aircraft : {self.name}  ({p.get('company','?')}, {p.get('country','?')})\n"
            f"  mass   = {p.get('mass',0):.1f} kg\n"
            f"  S      = {p.get('S',0):.2f} m²   b = {p.get('b',0):.2f} m\n"
            f"  U0     = {U0:.1f} m/s  ({U0*1.944:.1f} kn)"
        )


class AircraftFactory:
    """Create AircraftConfig from database, optionally overriding values."""

    @staticmethod
    def create(
        name: str,
        yaml_overrides: Optional[str] = None,
        param_overrides: Optional[Dict[str, Any]] = None,
    ) -> AircraftConfig:
        """
        Build an AircraftConfig.

        Parameters
        ----------
        name            : aircraft name (must be in the database)
        yaml_overrides  : path to a YAML file with parameter overrides
        param_overrides : dict of parameter overrides (highest priority)

        Returns
        -------
        AircraftConfig with merged aero_params
        """
        params = get_aircraft_params(name)

        # Apply YAML overrides
        if yaml_overrides and os.path.isfile(yaml_overrides):
            with open(yaml_overrides, "r") as f:
                ov = yaml.safe_load(f) or {}
            overrides = ov.get("overrides", ov)  # support both flat and nested
            params.update({k: v for k, v in overrides.items() if k in params})

        # Apply dict overrides (highest priority)
        if param_overrides:
            params.update({k: v for k, v in param_overrides.items() if k in params})

        return AircraftConfig(name=name, aero_params=params)

    @staticmethod
    def from_yaml(config_path: str) -> AircraftConfig:
        """
        Create an AircraftConfig from an aircraft.yaml config file.

        The YAML must contain at least:
            aircraft_name: <name>
        Optionally:
            overrides:
              mass: ...
        """
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f) or {}

        name      = cfg.get("aircraft_name", "TB2")
        overrides = cfg.get("overrides", {})
        return AircraftFactory.create(name, param_overrides=overrides or None)

    @staticmethod
    def export_ardupilot_params(
        name: str,
        output_path: str,
        control_yaml: Optional[str] = None,
    ) -> None:
        """
        Export aircraft + control parameters in ArduPilot .param file format.

        Parameters
        ----------
        name         : aircraft name
        output_path  : destination .param file path
        control_yaml : optional control_params.yaml path for PID gains
        """
        params: Dict[str, float] = {}

        # --- Aircraft physical parameters (ArduPilot naming) ---
        p = get_aircraft_params(name)
        params["MASS"]         = float(p["mass"])
        params["WING_AREA"]    = float(p["S"])
        params["WING_SPAN"]    = float(p["b"])
        params["MEAN_CHORD"]   = float(p["c"])
        params["IYY"]          = float(p["Iyy"])
        params["IXX"]          = float(p.get("ixx", 0))
        params["IZZ"]          = float(p.get("izz", 0))
        params["AIRSPEED_CRUISE"] = float(p["U0"])

        # --- Control parameters ---
        if control_yaml and os.path.isfile(control_yaml):
            with open(control_yaml, "r") as f:
                ctrl = yaml.safe_load(f) or {}
            params.update({k: float(v) for k, v in ctrl.items()
                           if isinstance(v, (int, float))})

        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        with open(output_path, "w") as f:
            f.write(f"# ArduPilot parameter file – {name}\n")
            for k, v in sorted(params.items()):
                f.write(f"{k},{v:.6f}\n")

        print(f"Exported {len(params)} parameters to {output_path}")
