"""
example_4_circuit_flight.py
===========================
Closed-loop AUTO mode: TB2 flying a rectangular four-leg circuit using
simple waypoint sequencing (NO polynomial trajectory).

The aircraft is commanded to fly to each waypoint in sequence; the L1
navigator switches to the next waypoint when within wp_switch_dist metres
(horizontal).  TECS handles altitude and airspeed control on each leg.

Circuit layout (NED, all at 100 m altitude):
  WP1: (   0,    0) – start / home
  WP2: ( 500,    0) – leg 1 (north)
  WP3: ( 500,  500) – leg 2 (east)
  WP4: (   0,  500) – leg 3 (south)
  WP5: (   0,    0) – leg 4 (west) / return

Output files (no GUI):
  Figures  → <script_dir>/../output/figures/
      example4_TB2_position_velocity.png
      example4_TB2_attitude_rates.png
      example4_TB2_controls.png
      example4_TB2_circuit_2d.png          (2-D top-down ground track)
      example4_TB2_altitude_throttle.png   (altitude & throttle vs time)
  CSV data → <script_dir>/../output/data/
      example4_TB2_circuit.csv
"""

import sys
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
from simulation.simulator import FixedWingSimulator

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
_HERE      = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(_HERE, "..", "output")
FIG_DIR    = os.path.join(OUTPUT_DIR, "figures")
DATA_DIR   = os.path.join(OUTPUT_DIR, "data")
FIG_DPI    = 150
FIG_FMT    = "png"

UAV_NAME        = "TB2"
CIRCUIT_ALT_M   = 100.0   # cruise altitude (m)
CIRCUIT_SIDE_M  = 2000.0  # square side length (m) – 2000m 边长，适合 TB2 级无人机
SIM_DURATION    = 900.0   # simulation duration (s) – 2000m边长每圈约200s，900s≈4.5圈
WP_SWITCH_DIST  = 350.0   # waypoint switching distance (m) – >R_min(233m)，提前切换最大限度减小超调
LOOP_CIRCUIT    = True    # continuously loop the circuit

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _makedirs():
    for d in (FIG_DIR, DATA_DIR):
        os.makedirs(d, exist_ok=True)

def _save(fig, name):
    path = os.path.join(FIG_DIR, f"{name}.{FIG_FMT}")
    try:
        fig.savefig(path, dpi=FIG_DPI, bbox_inches="tight")
        print(f"  [saved]  {path}")
    except Exception as e:
        print(f"  [ERROR]  {path}: {e}")
    finally:
        plt.close(fig)

# ===========================================================================
_makedirs()

# ---------------------------------------------------------------------------
# Build simulator
# ---------------------------------------------------------------------------
sim = FixedWingSimulator(
    aircraft_name=UAV_NAME,
    dt=0.01,
    duration=SIM_DURATION,
    initial_mode="AUTO",
    wind_type="NONE",
)

# ---------------------------------------------------------------------------
# Define four-leg rectangular circuit
# Waypoints: (north_m, east_m, alt_m)  – alt positive-up
# ---------------------------------------------------------------------------
S = CIRCUIT_SIDE_M
A = CIRCUIT_ALT_M
circuit_wps = [
    [0.0, 0.0, A],       # WP1  home / start
    [  S, 0.0, A],       # WP2  north leg end
    [  S,   S, A],       # WP3  east  leg end
    [0.0,   S, A],       # WP4  south leg end
    [0.0, 0.0, A],       # WP5  west  leg end / return to home
]

sim.wp_mgr.clear_waypoints()
for wp in circuit_wps:
    sim.wp_mgr.add_waypoint(*wp)

# ---------------------------------------------------------------------------
# Run – waypoint-sequencing mode (no polynomial trajectory)
# ---------------------------------------------------------------------------
print(f"\nRunning {UAV_NAME} four-leg circuit ({S:.0f} m square, "
      f"alt={A:.0f} m, dur={SIM_DURATION:.0f} s, "
      f"loop={'ON' if LOOP_CIRCUIT else 'OFF'}) …\n")

result = sim.run(
    closed_loop=True,
    use_trajectory=False,
    wp_switch_dist=WP_SWITCH_DIST,
    loop_circuit=LOOP_CIRCUIT,
)
print()
print(result.summary())
print()

# ---------------------------------------------------------------------------
# Post-process
# ---------------------------------------------------------------------------
h = result.history.to_dict()
t = h["t"]

# Save CSV
csv_path = os.path.join(DATA_DIR, f"example4_{UAV_NAME}_circuit.csv")
try:
    result.history.to_csv(csv_path)
    print(f"  [saved CSV]  {csv_path}")
except Exception as e:
    print(f"  [ERROR CSV]  {e}")

# ---------------------------------------------------------------------------
# Figure 1: Position & Velocity
# ---------------------------------------------------------------------------
fig1, axes = plt.subplots(2, 3, figsize=(15, 8))
fig1.suptitle(f"{UAV_NAME} – Circuit: Position & Velocity", fontsize=13)
_pairs1 = [
    ("North (m)",    h["x_north"]),
    ("East  (m)",    h["x_east"]),
    ("Altitude (m)", h["altitude"]),
    ("u (m/s)",      h["u"]),
    ("v (m/s)",      h["v"]),
    ("w (m/s)",      h["w"]),
]
for ax, (lbl, data) in zip(axes.flat, _pairs1):
    ax.plot(t, data, linewidth=1.2)
    ax.set_title(lbl)
    ax.set_xlabel("t (s)")
    ax.grid(True, alpha=0.3)
plt.tight_layout()
_save(fig1, f"example4_{UAV_NAME}_position_velocity")

# ---------------------------------------------------------------------------
# Figure 2: Attitude & Angular Rates
# ---------------------------------------------------------------------------
fig2, axes2 = plt.subplots(2, 3, figsize=(15, 8))
fig2.suptitle(f"{UAV_NAME} – Circuit: Attitude & Angular Rates", fontsize=13)
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
_save(fig2, f"example4_{UAV_NAME}_attitude_rates")

# ---------------------------------------------------------------------------
# Figure 3: Control Inputs
# ---------------------------------------------------------------------------
fig3, axes3 = plt.subplots(1, 4, figsize=(16, 4))
fig3.suptitle(f"{UAV_NAME} – Circuit: Control Inputs", fontsize=13)
for ax, lbl, key in zip(axes3,
                         ["Elevator", "Aileron", "Rudder", "Throttle"],
                         ["elevator", "aileron", "rudder", "throttle"]):
    ax.plot(t, h[key], linewidth=1.2)
    ax.set_title(lbl)
    ax.set_xlabel("t (s)")
    ax.grid(True, alpha=0.3)
plt.tight_layout()
_save(fig3, f"example4_{UAV_NAME}_controls")

# ---------------------------------------------------------------------------
# Figure 4: 2-D top-down ground track
# ---------------------------------------------------------------------------
fig4, ax4 = plt.subplots(figsize=(8, 8))
ax4.plot(h["x_east"], h["x_north"], color="steelblue", linewidth=1.5,
         label="Actual track")
ax4.scatter(h["x_east"][0],  h["x_north"][0],  color="green", s=80,
            zorder=5, label="Start")
ax4.scatter(h["x_east"][-1], h["x_north"][-1], color="red", s=80,
            zorder=5, label="End")

# Draw circuit reference (dashed)
_cwp_e = [w[1] for w in circuit_wps]
_cwp_n = [w[0] for w in circuit_wps]
ax4.plot(_cwp_e, _cwp_n, "k--", linewidth=1.0, alpha=0.5, label="Circuit ref")
for _i, (_ne, _nn) in enumerate(zip(_cwp_e, _cwp_n)):
    ax4.annotate(f"WP{_i+1}", xy=(_ne, _nn), fontsize=9,
                 xytext=(8, 8), textcoords="offset points")
    ax4.scatter(_ne, _nn, color="orange", marker="^", s=70, zorder=6)

ax4.set_xlabel("East (m)")
ax4.set_ylabel("North (m)")
ax4.set_title(f"{UAV_NAME} – 2-D Ground Track (circuit mode)")
ax4.set_aspect("equal")
ax4.legend(fontsize=9)
ax4.grid(True, alpha=0.3)
plt.tight_layout()
_save(fig4, f"example4_{UAV_NAME}_circuit_2d")

# ---------------------------------------------------------------------------
# Figure 5: Altitude & Throttle vs time
# ---------------------------------------------------------------------------
fig5, (ax5a, ax5b) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
fig5.suptitle(f"{UAV_NAME} – Circuit: Altitude & Throttle", fontsize=13)

ax5a.plot(t, h["altitude"], linewidth=1.2, color="steelblue", label="Altitude")
ax5a.axhline(A, color="tomato", linestyle="--", linewidth=1.0, label=f"Target {A:.0f} m")
ax5a.axvline(SIM_DURATION * 0.50, color="gray", linestyle=":", linewidth=0.8,
             label=f"SS window start (t={SIM_DURATION*0.50:.0f}s)")
ax5a.set_ylabel("Altitude (m)")
ax5a.legend(fontsize=9)
ax5a.grid(True, alpha=0.3)

ax5b.plot(t, h["throttle"], linewidth=1.2, color="darkorange", label="Throttle")
ax5b.set_ylabel("Throttle [0–1]")
ax5b.set_xlabel("t (s)")
ax5b.set_ylim(-0.05, 1.05)
ax5b.legend(fontsize=9)
ax5b.grid(True, alpha=0.3)

plt.tight_layout()
_save(fig5, f"example4_{UAV_NAME}_altitude_throttle")

# ---------------------------------------------------------------------------
# Performance metrics
# ---------------------------------------------------------------------------
alt_arr = np.array(h["altitude"])
# Steady-state: t > 50% of total sim time (跳过初始爬升和第一圈，覆盖完整多圈数据)
# 这种方式比"last 30s"更鲁棒：不受末段恰好处于转弯的影响
t_arr   = np.array(t)
ss_t0   = SIM_DURATION * 0.50        # 从50%时刻开始
mask_ss = t_arr >= ss_t0
alt_ss  = alt_arr[mask_ss]
alt_err = alt_arr - A
rms_err = float(np.sqrt(np.mean(alt_err**2)))
ss_std  = float(np.std(alt_ss))
ss_mean = float(np.mean(alt_ss))
peak_err = float(np.max(np.abs(alt_err)))

print("=" * 50)
print(f"  Altitude target  : {A:.1f} m")
print(f"  RMS alt error    : {rms_err:.2f} m  (full run)")
print(f"  Peak alt error   : {peak_err:.2f} m")
print(f"  SS mean alt      : {ss_mean:.2f} m  (t>{ss_t0:.0f}s)")
print(f"  SS std dev       : {ss_std:.3f} m  (t>{ss_t0:.0f}s, {mask_ss.sum()} pts)")
print(f"  Final position   : N={h['x_north'][-1]:.0f} m  E={h['x_east'][-1]:.0f} m")
print(f"  Final airspeed   : {h['airspeed'][-1]:.1f} m/s")
print("=" * 50)
print()
print(f"Done.  All outputs written to: {os.path.abspath(OUTPUT_DIR)}")
