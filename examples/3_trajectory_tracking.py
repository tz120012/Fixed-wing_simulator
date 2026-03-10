"""
example_3_trajectory_tracking.py
===================================
Closed-loop AUTO mode: TB2 tracking a minimum-snap square trajectory.

Demonstrates:
  - FixedWingSimulator in AUTO mode
  - WaypointManager with minimum_snap trajectory
  - 5-layer ArduPilot control chain
  - 3D trajectory + 6-DOF time-history plots

Output files (saved automatically, no GUI window will appear):
  Figures  → <script_dir>/../output/figures/
      example3_TB2_position_velocity.png      (position & velocity vs time)
      example3_TB2_attitude_rates.png         (attitude & angular rates vs time)
      example3_TB2_controls.png               (control surface deflections)
      example3_TB2_trajectory_3d.png          (3-D NED path)
  CSV data → <script_dir>/../output/data/
      example3_TB2_trajectory.csv             (all 23 state channels)

Configure OUTPUT_DIR below to change the save location.
"""

import sys
import os

# ── Non-interactive backend MUST be set before any other matplotlib import ──
import matplotlib
matplotlib.use("Agg")          # renders to file only, no GUI window
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
from simulation.simulator import FixedWingSimulator

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
_HERE       = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR  = os.path.join(_HERE, "..", "output")
FIG_DIR     = os.path.join(OUTPUT_DIR, "figures")
DATA_DIR    = os.path.join(OUTPUT_DIR, "data")
FIG_DPI     = 150
FIG_FORMAT  = "png"

UAV_NAME = "TB2"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _makedirs() -> None:
    for d in (FIG_DIR, DATA_DIR):
        os.makedirs(d, exist_ok=True)

def _save_fig(fig: plt.Figure, name: str) -> None:
    path = os.path.join(FIG_DIR, f"{name}.{FIG_FORMAT}")
    try:
        fig.savefig(path, dpi=FIG_DPI, bbox_inches="tight")
        print(f"  [saved figure]  {path}")
    except Exception as exc:
        print(f"  [ERROR saving figure {path}]: {exc}")
    finally:
        plt.close(fig)


# ===========================================================================
# Main
# ===========================================================================
_makedirs()

# ---- Create simulator ------------------------------------------------------
sim = FixedWingSimulator(
    aircraft_name=UAV_NAME,
    dt=0.01,
    duration=60.0,
    initial_mode="AUTO",
    wind_type="NONE",
    traj_type="minimum_snap",
)

# ---- Define square waypoints (alt=100 m, 500 m sides) ----------------------
wps_alt100 = [
    [0.0,   0.0,   100.0],
    [500.0, 0.0,   100.0],
    [500.0, 500.0, 100.0],
    [0.0,   500.0, 100.0],
    [0.0,   0.0,   100.0],
]
sim.wp_mgr.clear_waypoints()
for wp in wps_alt100:
    sim.wp_mgr.add_waypoint(*wp)

# ---- Run -------------------------------------------------------------------
print("Running closed-loop trajectory tracking …")
result = sim.run(closed_loop=True)
print(result.summary())
print()

h = result.history.to_dict()
t = h["t"]

# ---- Save CSV (all 23 channels via built-in method) ------------------------
csv_path = os.path.join(DATA_DIR, f"example3_{UAV_NAME}_trajectory.csv")
try:
    result.history.to_csv(csv_path)
    print(f"  [saved CSV]     {csv_path}")
except Exception as exc:
    print(f"  [ERROR saving CSV {csv_path}]: {exc}")

# ---- Figure 1: Position & Velocity -----------------------------------------
fig1, axes = plt.subplots(2, 3, figsize=(15, 8))
fig1.suptitle(f"{UAV_NAME} – Position & Velocity", fontsize=13)
_pairs1 = [
    ("North (m)",  h["x_north"]),
    ("East  (m)",  h["x_east"]),
    ("Altitude (m)", h["altitude"]),
    ("u (m/s)",    h["u"]),
    ("v (m/s)",    h["v"]),
    ("w (m/s)",    h["w"]),
]
for ax, (lbl, data) in zip(axes.flat, _pairs1):
    ax.plot(t, data, linewidth=1.2)
    ax.set_title(lbl)
    ax.set_xlabel("t (s)")
    ax.grid(True, alpha=0.3)
plt.tight_layout()
_save_fig(fig1, f"example3_{UAV_NAME}_position_velocity")

# ---- Figure 2: Attitude & Angular Rates ------------------------------------
fig2, axes2 = plt.subplots(2, 3, figsize=(15, 8))
fig2.suptitle(f"{UAV_NAME} – Attitude & Angular Rates", fontsize=13)
_pairs2 = [
    ("φ (deg)",   np.degrees(h["phi"])),
    ("θ (deg)",   np.degrees(h["theta"])),
    ("ψ (deg)",   np.degrees(h["psi"])),
    ("p (deg/s)", np.degrees(h["p"])),
    ("q (deg/s)", np.degrees(h["q"])),
    ("r (deg/s)", np.degrees(h["r"])),
]
for ax, (lbl, data) in zip(axes2.flat, _pairs2):
    ax.plot(t, data, linewidth=1.2)
    ax.set_title(lbl)
    ax.set_xlabel("t (s)")
    ax.grid(True, alpha=0.3)
plt.tight_layout()
_save_fig(fig2, f"example3_{UAV_NAME}_attitude_rates")

# ---- Figure 3: Control Inputs -----------------------------------------------
fig3, axes3 = plt.subplots(1, 4, figsize=(16, 4))
fig3.suptitle(f"{UAV_NAME} – Control Inputs", fontsize=13)
for ax, lbl, key in zip(axes3,
                         ["Elevator", "Aileron", "Rudder", "Throttle"],
                         ["elevator", "aileron", "rudder", "throttle"]):
    ax.plot(t, h[key], linewidth=1.2)
    ax.set_title(lbl)
    ax.set_xlabel("t (s)")
    ax.grid(True, alpha=0.3)
plt.tight_layout()
_save_fig(fig3, f"example3_{UAV_NAME}_controls")

# ---- Figure 4: 3-D Trajectory -----------------------------------------------
fig4 = plt.figure(figsize=(10, 8))
ax3d = fig4.add_subplot(111, projection="3d")

xE  = h["x_east"]
xN  = h["x_north"]
alt = h["altitude"]

ax3d.plot(xE, xN, alt, color="steelblue", linewidth=1.5, label="Actual")
ax3d.scatter(xE[0], xN[0], alt[0], color="green", s=60, zorder=5, label="Start")
ax3d.scatter(xE[-1], xN[-1], alt[-1], color="red", s=60, zorder=5, label="End")

# Desired trajectory (if logged)
if not np.all(h["des_north"] == 0):
    ax3d.plot(h["des_east"], h["des_north"], -h["des_down"],
              color="tomato", linewidth=1.2, linestyle="--", label="Desired")

# Mark waypoints
for wp in wps_alt100:
    ax3d.scatter(wp[1], wp[0], wp[2],   # east, north, alt
                 color="orange", marker="^", s=80, zorder=6)

ax3d.set_xlabel("East (m)")
ax3d.set_ylabel("North (m)")
ax3d.set_zlabel("Altitude (m)")
ax3d.set_title(f"{UAV_NAME} – 3D Trajectory (AUTO mode)")
ax3d.legend(fontsize=9)
plt.tight_layout()
_save_fig(fig4, f"example3_{UAV_NAME}_trajectory_3d")

print()
print("Done.  All outputs written to:", os.path.abspath(OUTPUT_DIR))
