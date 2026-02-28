"""
example_6_ardupilot_parameters.py
====================================
Load ArduPilot-format control parameters, adjust PID gains,
and compare tracking performance.

Demonstrates:
  - ArdupilotParams.from_yaml() / to_yaml()
  - AirdupilotParams.validate()
  - Hot-reload of gains during simulation
  - Export to .param file (ArduPilot compatible)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
import matplotlib.pyplot as plt

from control.ardupilot_compat import ArdupilotParams
from models.aircraft_factory  import AircraftFactory

# ---- Load default parameters -----------------------------------------------
config_dir = os.path.join(os.path.dirname(__file__), "..", "config")
ctrl_path  = os.path.join(config_dir, "control_params.yaml")

ap = ArdupilotParams.from_yaml(ctrl_path)
print("Loaded ArduPilot parameters:")
print(f"  PTCH_P      = {ap.PTCH_P}")
print(f"  PTCH_RATE_P = {ap.PTCH_RATE_P}")
print(f"  ROLL_P      = {ap.ROLL_P}")
print(f"  YAW_RATE_P  = {ap.YAW_RATE_P}")
print(f"  AIRSPEED_CRUISE = {ap.AIRSPEED_CRUISE} m/s")
print()

ok = ap.validate()
print(f"Parameter validation: {'PASSED' if ok else 'WARNINGS found'}")
print()

# ---- Export to ArduPilot .param file ----------------------------------------
out_path = os.path.join(os.path.dirname(__file__), "..", "config", "TB2_export.param")
AircraftFactory.export_ardupilot_params("TB2", out_path, ctrl_path)

# ---- Compare: default vs high-gain pitch controller -------------------------
from simulation.simulator import FixedWingSimulator

def run_with_gains(ptch_p: float, label: str):
    sim = FixedWingSimulator(
        aircraft_name="TB2",
        dt=0.01, duration=20.0,
        initial_mode="FBW_B",
        wind_type="NONE",
    )
    # Override pitch gain
    sim.ap_params.PTCH_P = ptch_p
    sim.att_ctrl.reload_gains(sim.ap_params)

    sim.wp_mgr.add_waypoint(0.0, 0.0, 100.0)
    sim.wp_mgr.add_waypoint(500.0, 0.0, 150.0)   # climb request

    result = sim.run(closed_loop=True)
    h = result.history.to_dict()
    return h["t"], h["altitude"], h["theta"], label

results_ap = [
    run_with_gains(0.5,  "PTCH_P=0.5 (low)"),
    run_with_gains(1.0,  "PTCH_P=1.0 (default)"),
    run_with_gains(2.0,  "PTCH_P=2.0 (high)"),
]

# ---- Plot -------------------------------------------------------------------
fig, axes = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle("ArduPilot Parameter Sensitivity – PTCH_P gain")

for t, alt, theta, lbl in results_ap:
    axes[0].plot(t, alt,              label=lbl, linewidth=1.5)
    axes[1].plot(t, np.degrees(theta), label=lbl, linewidth=1.5)

axes[0].axhline(100, color="k", linestyle="--", alpha=0.4, label="Target 100m")
axes[0].set_ylabel("Altitude (m)"); axes[0].legend(fontsize=9); axes[0].grid(True, alpha=0.3)
axes[1].set_ylabel("Pitch θ (deg)"); axes[1].set_xlabel("Time (s)"); axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
