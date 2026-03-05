"""
4_circuit_flight_wind.py
========================
在 example4 四边形航线基础上加入风场测试，对比三种风场条件：
  - NONE      : 无风（基线）
  - FIXED     : 常值侧风 8 m/s（来自西，270°）
  - COMBINED  : 常值风 8 m/s + Dryden 中等湍流 + 离散阵风

三种条件复用同一组航点和仿真参数，在同一幅图上叠加轨迹对比。

Output files → <script_dir>/../output/figures/
    example4w_TB2_circuit_2d.png          2-D 轨迹对比（含风箭头）
    example4w_TB2_altitude.png            高度随时间对比
    example4w_TB2_airspeed.png            空速随时间对比
    example4w_TB2_controls_combined.png   COMBINED 条件控制量
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
# Paths
# ---------------------------------------------------------------------------
_HERE      = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(_HERE, "..", "output")
FIG_DIR    = os.path.join(OUTPUT_DIR, "figures")
DATA_DIR   = os.path.join(OUTPUT_DIR, "data")
FIG_DPI    = 150
FIG_FMT    = "png"
os.makedirs(FIG_DIR, exist_ok=True)
os.makedirs(DATA_DIR, exist_ok=True)

def _save(fig, name):
    path = os.path.join(FIG_DIR, f"{name}.{FIG_FMT}")
    fig.savefig(path, dpi=FIG_DPI, bbox_inches="tight")
    print(f"  [saved]  {path}")
    plt.close(fig)

# ---------------------------------------------------------------------------
# Common simulation parameters
# ---------------------------------------------------------------------------
UAV_NAME       = "TB2"
CIRCUIT_ALT_M  = 100.0
CIRCUIT_SIDE_M = 2000.0
SIM_DURATION   = 900.0
WP_SWITCH_DIST = 350.0

S = CIRCUIT_SIDE_M
A = CIRCUIT_ALT_M
circuit_wps = [
    [0.0, 0.0, A],
    [  S, 0.0, A],
    [  S,   S, A],
    [0.0,   S, A],
    [0.0, 0.0, A],
]

WIND_SPEED_MPS  = 8.0
WIND_DIR_DEG    = 270.0   # from west

# 离散阵风列表（COMBINED 条件用）
GUSTS = [
    {"axis": 0, "amplitude":  6.0, "gradient_m": 300.0, "t_start": 120.0},  # North 阵风
    {"axis": 1, "amplitude":  4.0, "gradient_m": 200.0, "t_start": 300.0},  # East  阵风
    {"axis": 2, "amplitude":  3.0, "gradient_m": 150.0, "t_start": 500.0},  # Down  阵风
    {"axis": 0, "amplitude": -5.0, "gradient_m": 250.0, "t_start": 680.0},  # North 反向阵风
]

# ---------------------------------------------------------------------------
# Wind scenarios  {label: constructor_kwargs}
# ---------------------------------------------------------------------------
SCENARIOS = {
    "NONE": dict(
        wind_type="NONE",
    ),
    "FIXED 8 m/s W": dict(
        wind_type="FIXED",
        wind_speed=WIND_SPEED_MPS,
        wind_dir_deg=WIND_DIR_DEG,
    ),
    "COMBINED (Fixed+Dryden+Gust)": dict(
        wind_type="COMBINED",
        wind_speed=WIND_SPEED_MPS,
        wind_dir_deg=WIND_DIR_DEG,
        wind_severity="moderate",
        wind_gusts=GUSTS,
    ),
}

COLORS = {"NONE": "steelblue",
          "FIXED 8 m/s W": "darkorange",
          "COMBINED (Fixed+Dryden+Gust)": "crimson"}
LSTYLE = {"NONE": "-", "FIXED 8 m/s W": "--", "COMBINED (Fixed+Dryden+Gust)": ":"}

# ---------------------------------------------------------------------------
# Run all scenarios
# ---------------------------------------------------------------------------
results = {}
for label, wind_kwargs in SCENARIOS.items():
    print(f"\n{'='*60}")
    print(f"  Scenario: {label}")
    print(f"{'='*60}")

    sim = FixedWingSimulator(
        aircraft_name=UAV_NAME,
        dt=0.01,
        duration=SIM_DURATION,
        initial_mode="AUTO",
        **wind_kwargs,
    )
    sim.wp_mgr.clear_waypoints()
    for wp in circuit_wps:
        sim.wp_mgr.add_waypoint(*wp)

    res = sim.run(
        closed_loop=True,
        use_trajectory=False,
        wp_switch_dist=WP_SWITCH_DIST,
        loop_circuit=True,
    )
    results[label] = res.history.to_dict()

    # save CSV
    csv_path = os.path.join(DATA_DIR, f"example4w_TB2_{label.split()[0].lower()}.csv")
    try:
        res.history.to_csv(csv_path)
        print(f"  [CSV]  {csv_path}")
    except Exception as e:
        print(f"  [CSV error]  {e}")

    # quick metrics
    t_arr   = np.array(results[label]["t"])
    alt_arr = np.array(results[label]["altitude"])
    mask_ss = t_arr >= SIM_DURATION * 0.5
    ss_std  = np.std(alt_arr[mask_ss])
    ss_mean = np.mean(alt_arr[mask_ss])
    rms_err = np.sqrt(np.mean((alt_arr - A) ** 2))
    print(f"  SS alt mean={ss_mean:.2f} m  std={ss_std:.3f} m  RMS_err={rms_err:.2f} m")

print()

# ---------------------------------------------------------------------------
# Figure 1: 2-D Ground Track comparison
# ---------------------------------------------------------------------------
fig1, ax1 = plt.subplots(figsize=(9, 9))

for label, h in results.items():
    ax1.plot(h["x_east"], h["x_north"],
             color=COLORS[label], linestyle=LSTYLE[label],
             linewidth=1.5, alpha=0.85, label=label)

# Reference circuit
_cwp_e = [w[1] for w in circuit_wps]
_cwp_n = [w[0] for w in circuit_wps]
ax1.plot(_cwp_e, _cwp_n, "k--", linewidth=1.0, alpha=0.4, label="Circuit ref")
for _i, (_ne, _nn) in enumerate(zip(_cwp_e, _cwp_n)):
    ax1.scatter(_ne, _nn, color="orange", marker="^", s=70, zorder=6)
    ax1.annotate(f"WP{_i+1}", xy=(_ne, _nn), fontsize=9,
                 xytext=(8, 8), textcoords="offset points")

# Wind arrow (FIXED / COMBINED scenarios use same mean wind direction)
# Met convention: wind FROM direction → arrow points INTO the scene
ax_center_e = S / 2
ax_center_n = S / 2
# Wind FROM 270° (west) → blows EAST → arrow dir = East (+E)
import math
wdir_rad = math.radians(WIND_DIR_DEG)  # met: from this direction
arrow_de = -WIND_SPEED_MPS * 20 * math.sin(wdir_rad)   # component east
arrow_dn = -WIND_SPEED_MPS * 20 * math.cos(wdir_rad)   # component north
ax1.annotate("", xy=(ax_center_e + arrow_de, ax_center_n + arrow_dn),
             xytext=(ax_center_e, ax_center_n),
             arrowprops=dict(arrowstyle="->", color="gray", lw=2))
ax1.text(ax_center_e + arrow_de * 0.5, ax_center_n + arrow_dn * 0.5 + 60,
         f"Wind {WIND_SPEED_MPS:.0f} m/s\n({WIND_DIR_DEG:.0f}°)", ha="center",
         fontsize=9, color="gray")

ax1.set_xlabel("East (m)")
ax1.set_ylabel("North (m)")
ax1.set_title(f"{UAV_NAME} – Circuit Ground Track: Wind Comparison")
ax1.set_aspect("equal")
ax1.legend(fontsize=9, loc="upper right")
ax1.grid(True, alpha=0.3)
plt.tight_layout()
_save(fig1, "example4w_TB2_circuit_2d")

# ---------------------------------------------------------------------------
# Figure 2: Altitude vs time
# ---------------------------------------------------------------------------
fig2, ax2 = plt.subplots(figsize=(13, 5))
for label, h in results.items():
    ax2.plot(h["t"], h["altitude"],
             color=COLORS[label], linestyle=LSTYLE[label],
             linewidth=1.2, alpha=0.9, label=label)
ax2.axhline(A, color="black", linestyle="--", linewidth=0.8, alpha=0.5,
            label=f"Target {A:.0f} m")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Altitude (m)")
ax2.set_title(f"{UAV_NAME} – Altitude: Wind Comparison")
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3)
plt.tight_layout()
_save(fig2, "example4w_TB2_altitude")

# ---------------------------------------------------------------------------
# Figure 3: Airspeed vs time
# ---------------------------------------------------------------------------
fig3, ax3 = plt.subplots(figsize=(13, 5))
for label, h in results.items():
    ax3.plot(h["t"], h["airspeed"],
             color=COLORS[label], linestyle=LSTYLE[label],
             linewidth=1.2, alpha=0.9, label=label)
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Airspeed (m/s)")
ax3.set_title(f"{UAV_NAME} – Airspeed: Wind Comparison")
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)
plt.tight_layout()
_save(fig3, "example4w_TB2_airspeed")

# ---------------------------------------------------------------------------
# Figure 4: Control inputs for COMBINED scenario
# ---------------------------------------------------------------------------
h_c = results["COMBINED (Fixed+Dryden+Gust)"]
fig4, axes4 = plt.subplots(2, 2, figsize=(13, 8))
fig4.suptitle(f"{UAV_NAME} – Controls under COMBINED Wind\n"
              f"(Fixed {WIND_SPEED_MPS:.0f} m/s + Dryden moderate + Gusts)", fontsize=12)
for ax, (key, lbl) in zip(axes4.flat, [
    ("elevator", "Elevator"),
    ("aileron",  "Aileron"),
    ("rudder",   "Rudder"),
    ("throttle", "Throttle"),
]):
    ax.plot(h_c["t"], h_c[key], linewidth=0.9, color="crimson")
    # overlay NONE baseline
    h_n = results["NONE"]
    ax.plot(h_n["t"], h_n[key], linewidth=0.8, color="steelblue", alpha=0.5,
            linestyle="--", label="NONE (baseline)")
    ax.set_title(lbl)
    ax.set_xlabel("t (s)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

# mark gust times
for ax in axes4.flat:
    for g in GUSTS:
        ax.axvline(g["t_start"], color="gray", linewidth=0.8,
                   linestyle=":", alpha=0.6)

plt.tight_layout()
_save(fig4, "example4w_TB2_controls_combined")

# ---------------------------------------------------------------------------
# Figure 5: North/East position comparison (drift from wind)
# ---------------------------------------------------------------------------
fig5, (axN, axE) = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
fig5.suptitle(f"{UAV_NAME} – Position Drift: Wind Comparison", fontsize=12)
for label, h in results.items():
    axN.plot(h["t"], h["x_north"], color=COLORS[label], linestyle=LSTYLE[label],
             linewidth=1.2, alpha=0.9, label=label)
    axE.plot(h["t"], h["x_east"],  color=COLORS[label], linestyle=LSTYLE[label],
             linewidth=1.2, alpha=0.9, label=label)
axN.set_ylabel("North (m)")
axN.legend(fontsize=9)
axN.grid(True, alpha=0.3)
axE.set_ylabel("East (m)")
axE.set_xlabel("Time (s)")
axE.legend(fontsize=9)
axE.grid(True, alpha=0.3)
plt.tight_layout()
_save(fig5, "example4w_TB2_position_comparison")

print(f"\nDone.  All outputs written to: {os.path.abspath(FIG_DIR)}")
