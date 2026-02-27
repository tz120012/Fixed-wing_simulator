"""
example_1_linear_response.py
==============================
4-DOF linear model pulse response + closed-loop PID comparison.

Demonstrates:
  - LinearModel.run_analysis()         – open-loop modal analysis
  - Modal analysis (Short Period, Phugoid)
  - FixedWingSimulator in FBW_B mode   – closed-loop PID step response
  - Overlay: open-loop vs closed-loop pitch-rate & theta

Output files (saved automatically, no GUI window will appear):
  Figures  → <script_dir>/../output/figures/
      example1_TB2_openloop_vs_closedloop.png
  CSV data → <script_dir>/../output/data/
      example1_TB2_linear_openloop.csv
      example1_TB2_closedloop_fbwb.csv

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

from models.aircraft_database import get_aircraft_params
from dynamics.linear_model    import LinearModel
from simulation.simulator     import FixedWingSimulator

# ---------------------------------------------------------------------------
# Configuration – change OUTPUT_DIR to redirect all saved files
# ---------------------------------------------------------------------------
_HERE       = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR  = os.path.join(_HERE, "..", "output")   # <project>/output/
FIG_DIR     = os.path.join(OUTPUT_DIR, "figures")
DATA_DIR    = os.path.join(OUTPUT_DIR, "data")
FIG_DPI     = 150          # PNG resolution (dots per inch)
FIG_FORMAT  = "png"        # "png" or "jpg"

UAV_NAME = "TB2"

# ---------------------------------------------------------------------------
# Helper – create directories safely
# ---------------------------------------------------------------------------
def _makedirs() -> None:
    for d in (FIG_DIR, DATA_DIR):
        os.makedirs(d, exist_ok=True)

# ---------------------------------------------------------------------------
# Helper – save figure with error handling
# ---------------------------------------------------------------------------
def _save_fig(fig: plt.Figure, name: str) -> None:
    path = os.path.join(FIG_DIR, f"{name}.{FIG_FORMAT}")
    try:
        fig.savefig(path, dpi=FIG_DPI, bbox_inches="tight")
        print(f"  [saved figure]  {path}")
    except Exception as exc:
        print(f"  [ERROR saving figure {path}]: {exc}")
    finally:
        plt.close(fig)

# ---------------------------------------------------------------------------
# Helper – save numpy arrays to CSV with error handling
# ---------------------------------------------------------------------------
def _save_csv(path: str, header: list, arrays: list) -> None:
    try:
        data = np.column_stack(arrays)
        np.savetxt(path, data, delimiter=",",
                   header=",".join(header), comments="")
        print(f"  [saved CSV]     {path}")
    except Exception as exc:
        print(f"  [ERROR saving CSV {path}]: {exc}")


# ===========================================================================
# Main
# ===========================================================================
_makedirs()

# ===========================================================================
# Part 1: Open-loop linear analysis
# ===========================================================================
print("=" * 55)
print("Part 1 – Open-loop 4-DOF linear analysis")
print("=" * 55)

params = get_aircraft_params(UAV_NAME)
model  = LinearModel(params)
pulses = [{"start_time": 1.0, "duration": 0.5, "angle_deg": 2.0}]

result = model.run_analysis(pulses, duration=10.0, uav_name=UAV_NAME)

print(result.summary())
print()
for mode in result.modes:
    print(f"  {mode}")
print()

t_ol     = result.t
q_ol     = np.degrees(result.y[2])   # pitch rate  (deg/s)
theta_ol = np.degrees(result.y[3])   # pitch angle (deg)
de_ol    = np.degrees(result.de)     # elevator input (deg)

# Save open-loop CSV
# Columns: t, u_p, alpha_deg, q_deg_s, theta_deg, elevator_deg
_save_csv(
    os.path.join(DATA_DIR, f"example1_{UAV_NAME}_linear_openloop.csv"),
    header=["t_s", "u_p", "alpha_deg", "q_deg_s", "theta_deg", "elevator_deg"],
    arrays=[t_ol,
            result.y[0],
            np.degrees(result.y[1]),
            q_ol,
            theta_ol,
            de_ol],
)

# ===========================================================================
# Part 2: Closed-loop PID step response (FBW_B mode)
# ===========================================================================
print("=" * 55)
print("Part 2 – Closed-loop PID step response (FBW_B)")
print("=" * 55)

sim = FixedWingSimulator(
    aircraft_name="TB2",
    dt=0.01,
    duration=15.0,
    initial_mode="FBW_B",   # altitude + airspeed hold, full PID active
    wind_type="NONE",
)

# Waypoint: climb from trim altitude to 80 m (triggers pitch PID step)
sim.wp_mgr.add_waypoint(0.0, 0.0,   0.0)
sim.wp_mgr.add_waypoint(0.0, 0.0,  80.0)

cl_result = sim.run(closed_loop=True)
h_cl = cl_result.history.to_dict()

t_cl     = h_cl["t"]
theta_cl = np.degrees(h_cl["theta"])
q_cl     = np.degrees(h_cl["q"])
alt_cl   = h_cl["altitude"]
elev_cl  = h_cl["elevator"]

print(f"  Max pitch angle : {theta_cl.max():.2f} deg")
print(f"  Final altitude  : {alt_cl[-1]:.1f} m  (target 80 m)")
print()

# Save closed-loop CSV via built-in to_csv()
csv_cl = os.path.join(DATA_DIR, f"example1_{UAV_NAME}_closedloop_fbwb.csv")
try:
    cl_result.history.to_csv(csv_cl)
    print(f"  [saved CSV]     {csv_cl}")
except Exception as exc:
    print(f"  [ERROR saving CSV {csv_cl}]: {exc}")

# ===========================================================================
# Part 3: Overlay plot  (saved to PNG, no window)
# ===========================================================================
fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
fig.suptitle(f"{UAV_NAME}  –  Open-loop (linear) vs Closed-loop (PID, FBW_B)")

# Subplot 1: open-loop pitch rate
axes[0].plot(t_ol, q_ol, color="steelblue", linewidth=1.5,
             label="Open-loop q (linear, 2° pulse)")
axes[0].axhline(0, color="k", linewidth=0.5)
axes[0].set_ylabel("Pitch rate q (deg/s)")
axes[0].legend(fontsize=9)
axes[0].grid(True, alpha=0.3)
axes[0].set_title("Open-loop: elevator pulse response (no controller)")

# Subplot 2: open-loop vs closed-loop pitch angle
axes[1].plot(t_ol, theta_ol, color="steelblue", linewidth=1.5,
             label="Open-loop θ (linear)")
axes[1].plot(t_cl, theta_cl, color="tomato",    linewidth=1.5,
             label="Closed-loop θ (PID, FBW_B)")
axes[1].set_ylabel("Pitch angle θ (deg)")
axes[1].legend(fontsize=9)
axes[1].grid(True, alpha=0.3)
axes[1].set_title("Pitch angle: open-loop vs PID-controlled")

# Subplot 3: closed-loop altitude tracking
axes[2].plot(t_cl, alt_cl, color="seagreen", linewidth=1.5,
             label="Altitude (closed-loop)")
axes[2].axhline(80.0, color="k", linestyle="--", alpha=0.5,
                label="Target 80 m")
axes[2].set_ylabel("Altitude (m)")
axes[2].set_xlabel("Time (s)")
axes[2].legend(fontsize=9)
axes[2].grid(True, alpha=0.3)
axes[2].set_title("Closed-loop altitude step response")

plt.tight_layout()
_save_fig(fig, f"example1_{UAV_NAME}_openloop_vs_closedloop")

print()
print("Done.  All outputs written to:", os.path.abspath(OUTPUT_DIR))
