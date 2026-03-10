"""
example_2_nonlinear_dynamics.py
=================================
6-DOF nonlinear dynamics: open-loop pulse + closed-loop PID comparison.

Demonstrates:
  - NonlinearModel.compute_trim()  – level-flight trim computation
  - NonlinearModel.simulate()      – open-loop aileron pulse (no controller)
  - FixedWingSimulator STABILIZE   – closed-loop PID roll/pitch stabilisation
  - Side-by-side roll response: open-loop vs PID-controlled

Output files (saved automatically, no GUI window will appear):
  Figures  → <script_dir>/../output/figures/
      example2_Predator_openloop_vs_closedloop.png
  CSV data → <script_dir>/../output/data/
      example2_Predator_nonlinear_openloop.csv
      example2_Predator_closedloop_stabilize.csv

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
from dynamics.nonlinear_model import NonlinearModel
from simulation.simulator     import FixedWingSimulator

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
# Part 1: Open-loop nonlinear simulation
# ===========================================================================
print("=" * 55)
print("Part 1 – Open-loop 6-DOF nonlinear simulation")
print("=" * 55)

params = get_aircraft_params(UAV_NAME)
model  = NonlinearModel(params)

trim = model.compute_trim()
print(f"Trim speed  : {trim.U0:.2f} m/s ({trim.U0 * 1.944:.2f} kn)")
print(f"Trim alpha  : {np.degrees(trim.alpha_trim):.3f} deg")
print(f"Trim delta_e: {np.degrees(trim.de_trim):.3f} deg")

# 5-deg aileron pulse at t=2s (open-loop, no controller)
pulses_ol = [
    {"start_time": 2.0, "duration": 0.5, "roll_deg": 5.0,
     "angle_deg": 0.0, "yaw_deg": 0.0, "throttle": 1.0},
]
result_ol = model.simulate(pulses_ol, duration=15.0, n_points=1000)
print(result_ol.summary())

t_ol    = result_ol.t
phi_ol  = np.degrees(result_ol.y[6])    # roll angle  (deg)
p_ol    = np.degrees(result_ol.y[3])    # roll rate   (deg/s)
theta_ol = np.degrees(result_ol.y[7])   # pitch angle (deg)

# Save open-loop CSV
# State order: [u,v,w, p,q,r, phi,theta,psi, x_north,x_east,x_down]
state_labels = ["u_m_s", "v_m_s", "w_m_s",
                "p_rad_s", "q_rad_s", "r_rad_s",
                "phi_rad", "theta_rad", "psi_rad",
                "x_north_m", "x_east_m", "x_down_m"]
_save_csv(
    os.path.join(DATA_DIR, f"example2_{UAV_NAME}_nonlinear_openloop.csv"),
    header=["t_s"] + state_labels,
    arrays=[t_ol] + [result_ol.y[i] for i in range(12)],
)

# ===========================================================================
# Part 2: Closed-loop PID (STABILIZE mode)
# ===========================================================================
print()
print("=" * 55)
print("Part 2 – Closed-loop PID simulation (STABILIZE)")
print("=" * 55)

sim = FixedWingSimulator(
    aircraft_name="TB2",
    dt=0.01,
    duration=15.0,
    initial_mode="STABILIZE",   # full attitude PID active
    wind_type="NONE",
)
sim.wp_mgr.add_waypoint(0.0, 0.0, 100.0)   # hold altitude 100 m

cl_result = sim.run(closed_loop=True)
h_cl = cl_result.history.to_dict()

t_cl     = h_cl["t"]
phi_cl   = np.degrees(h_cl["phi"])
p_cl     = np.degrees(h_cl["p"])
theta_cl = np.degrees(h_cl["theta"])
alt_cl   = h_cl["altitude"]

print(f"  Max |roll|  open-loop  : {np.abs(phi_ol).max():.2f} deg")
print(f"  Max |roll|  closed-loop: {np.abs(phi_cl).max():.2f} deg")
print(f"  Final roll  closed-loop: {phi_cl[-1]:.2f} deg  (target ~0)")
print()

# Save closed-loop CSV via built-in to_csv()
csv_cl = os.path.join(DATA_DIR, f"example2_{UAV_NAME}_closedloop_stabilize.csv")
try:
    cl_result.history.to_csv(csv_cl)
    print(f"  [saved CSV]     {csv_cl}")
except Exception as exc:
    print(f"  [ERROR saving CSV {csv_cl}]: {exc}")

# ===========================================================================
# Part 3: Side-by-side comparison plot  (saved to PNG, no window)
# ===========================================================================
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle(f"{UAV_NAME}  –  Open-loop (6-DOF) vs Closed-loop (PID, STABILIZE)")

# Roll angle
axes[0, 0].plot(t_ol, phi_ol, color="steelblue", linewidth=1.5)
axes[0, 0].axhline(0, color="k", linewidth=0.5, linestyle="--")
axes[0, 0].set_title("Open-loop: Roll angle φ")
axes[0, 0].set_ylabel("φ (deg)")
axes[0, 0].grid(True, alpha=0.3)

axes[0, 1].plot(t_cl, phi_cl, color="tomato", linewidth=1.5)
axes[0, 1].axhline(0, color="k", linewidth=0.5, linestyle="--")
axes[0, 1].set_title("Closed-loop PID: Roll angle φ")
axes[0, 1].set_ylabel("φ (deg)")
axes[0, 1].grid(True, alpha=0.3)

# Roll rate
axes[1, 0].plot(t_ol, p_ol, color="steelblue", linewidth=1.5)
axes[1, 0].axhline(0, color="k", linewidth=0.5, linestyle="--")
axes[1, 0].set_title("Open-loop: Roll rate p")
axes[1, 0].set_ylabel("p (deg/s)")
axes[1, 0].grid(True, alpha=0.3)

axes[1, 1].plot(t_cl, p_cl, color="tomato", linewidth=1.5)
axes[1, 1].axhline(0, color="k", linewidth=0.5, linestyle="--")
axes[1, 1].set_title("Closed-loop PID: Roll rate p")
axes[1, 1].set_ylabel("p (deg/s)")
axes[1, 1].grid(True, alpha=0.3)

# Pitch (OL) / Altitude (CL)
axes[2, 0].plot(t_ol, theta_ol, color="steelblue", linewidth=1.5)
axes[2, 0].set_title("Open-loop: Pitch angle θ")
axes[2, 0].set_ylabel("θ (deg)")
axes[2, 0].set_xlabel("Time (s)")
axes[2, 0].grid(True, alpha=0.3)

axes[2, 1].plot(t_cl, alt_cl, color="seagreen", linewidth=1.5,
                label="Altitude")
axes[2, 1].axhline(100.0, color="k", linestyle="--", alpha=0.5,
                   label="Target 100 m")
axes[2, 1].set_title("Closed-loop: Altitude hold")
axes[2, 1].set_ylabel("Altitude (m)")
axes[2, 1].set_xlabel("Time (s)")
axes[2, 1].legend(fontsize=9)
axes[2, 1].grid(True, alpha=0.3)

plt.tight_layout()
_save_fig(fig, f"example2_{UAV_NAME}_openloop_vs_closedloop")

print()
print("Done.  All outputs written to:", os.path.abspath(OUTPUT_DIR))
