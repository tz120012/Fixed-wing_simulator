"""
example_8_controller_comparison.py
===================================
对比 PX4 和 ArduPilot 两种姿态控制架构在相同航线上的性能。

测试场景：
- 相同的矩形航线（2000m × 2000m，高度 100m）
- 相同的飞行条件（无风）
- 相同的飞机模型（TB2）
- 不同的控制器架构

对比指标：
- 航迹跟踪精度
- 高度保持精度
- 姿态响应特性
- 控制输入平滑度

输出文件：
  Figures  → <script_dir>/../output/figures/
      example8_comparison_ground_track.png     (航迹对比)
      example8_comparison_altitude.png         (高度对比)
      example8_comparison_attitude.png         (姿态对比)
      example8_comparison_controls.png         (控制输入对比)
      example8_comparison_metrics.png          (性能指标对比)
  CSV data → <script_dir>/../output/data/
      example8_ardupilot_circuit.csv
      example8_px4_circuit.csv
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
CIRCUIT_ALT_M   = 100.0
CIRCUIT_SIDE_M  = 2000.0
SIM_DURATION    = 600.0   # 10分钟，约3圈
WP_SWITCH_DIST  = 350.0
LOOP_CIRCUIT    = True

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

def compute_metrics(history, target_alt, ss_start_ratio=0.5):
    """计算性能指标"""
    h = history.to_dict()
    t = np.array(h["t"])
    alt = np.array(h["altitude"])
    
    # 稳态区间（跳过初始爬升）
    ss_t0 = t[-1] * ss_start_ratio
    mask_ss = t >= ss_t0
    alt_ss = alt[mask_ss]
    
    # 高度误差
    alt_err = alt - target_alt
    rms_err = float(np.sqrt(np.mean(alt_err**2)))
    ss_std = float(np.std(alt_ss))
    ss_mean = float(np.mean(alt_ss))
    peak_err = float(np.max(np.abs(alt_err)))
    
    # 姿态变化率（平滑度指标）
    phi = np.array(h["phi"])
    theta = np.array(h["theta"])
    dt = np.diff(t)
    dphi_dt = np.diff(phi) / dt
    dtheta_dt = np.diff(theta) / dt
    roll_rate_std = float(np.std(dphi_dt))
    pitch_rate_std = float(np.std(dtheta_dt))
    
    # 控制输入变化率（平滑度指标）
    elevator = np.array(h["elevator"])
    aileron = np.array(h["aileron"])
    delevator_dt = np.diff(elevator) / dt
    daileron_dt = np.diff(aileron) / dt
    elevator_rate_std = float(np.std(delevator_dt))
    aileron_rate_std = float(np.std(daileron_dt))
    
    return {
        'rms_alt_error': rms_err,
        'peak_alt_error': peak_err,
        'ss_alt_mean': ss_mean,
        'ss_alt_std': ss_std,
        'roll_rate_std': roll_rate_std,
        'pitch_rate_std': pitch_rate_std,
        'elevator_rate_std': elevator_rate_std,
        'aileron_rate_std': aileron_rate_std,
    }

# ===========================================================================
_makedirs()

# ---------------------------------------------------------------------------
# 定义航线
# ---------------------------------------------------------------------------
S = CIRCUIT_SIDE_M
A = CIRCUIT_ALT_M
circuit_wps = [
    [0.0, 0.0, A],
    [  S, 0.0, A],
    [  S,   S, A],
    [0.0,   S, A],
    [0.0, 0.0, A],
]

# ---------------------------------------------------------------------------
# 测试 1: ArduPilot 控制器
# ---------------------------------------------------------------------------
print("=" * 70)
print("测试 1: ArduPilot 控制器")
print("=" * 70)

sim_ap = FixedWingSimulator(
    aircraft_name=UAV_NAME,
    dt=0.01,
    duration=SIM_DURATION,
    initial_mode="AUTO",
    wind_type="NONE",
    controller_type="ardupilot",  # 使用 ArduPilot
)

sim_ap.wp_mgr.clear_waypoints()
for wp in circuit_wps:
    sim_ap.wp_mgr.add_waypoint(*wp)

print(f"\n运行 ArduPilot 控制器测试...")
result_ap = sim_ap.run(
    closed_loop=True,
    use_trajectory=False,
    wp_switch_dist=WP_SWITCH_DIST,
    loop_circuit=LOOP_CIRCUIT,
)
print(result_ap.summary())

# 保存数据
csv_path_ap = os.path.join(DATA_DIR, "example8_ardupilot_circuit.csv")
result_ap.history.to_csv(csv_path_ap)
print(f"  [saved CSV]  {csv_path_ap}")

# 计算指标
metrics_ap = compute_metrics(result_ap.history, A)
print("\nArduPilot 性能指标:")
for key, val in metrics_ap.items():
    print(f"  {key:20s}: {val:.4f}")

# ---------------------------------------------------------------------------
# 测试 2: PX4 控制器
# ---------------------------------------------------------------------------
print("\n" + "=" * 70)
print("测试 2: PX4 控制器")
print("=" * 70)

sim_px4 = FixedWingSimulator(
    aircraft_name=UAV_NAME,
    dt=0.01,
    duration=SIM_DURATION,
    initial_mode="AUTO",
    wind_type="NONE",
    controller_type="px4",  # 使用 PX4
)

sim_px4.wp_mgr.clear_waypoints()
for wp in circuit_wps:
    sim_px4.wp_mgr.add_waypoint(*wp)

print(f"\n运行 PX4 控制器测试...")
result_px4 = sim_px4.run(
    closed_loop=True,
    use_trajectory=False,
    wp_switch_dist=WP_SWITCH_DIST,
    loop_circuit=LOOP_CIRCUIT,
)
print(result_px4.summary())

# 保存数据
csv_path_px4 = os.path.join(DATA_DIR, "example8_px4_circuit.csv")
result_px4.history.to_csv(csv_path_px4)
print(f"  [saved CSV]  {csv_path_px4}")

# 计算指标
metrics_px4 = compute_metrics(result_px4.history, A)
print("\nPX4 性能指标:")
for key, val in metrics_px4.items():
    print(f"  {key:20s}: {val:.4f}")

# ---------------------------------------------------------------------------
# 对比可视化
# ---------------------------------------------------------------------------
h_ap = result_ap.history.to_dict()
h_px4 = result_px4.history.to_dict()
t_ap = h_ap["t"]
t_px4 = h_px4["t"]

# ---------------------------------------------------------------------------
# Figure 1: 航迹对比（2D俯视图）
# ---------------------------------------------------------------------------
fig1, ax1 = plt.subplots(figsize=(10, 10))
ax1.plot(h_ap["x_east"], h_ap["x_north"], 
         color="steelblue", linewidth=1.5, label="ArduPilot", alpha=0.8)
ax1.plot(h_px4["x_east"], h_px4["x_north"], 
         color="darkorange", linewidth=1.5, label="PX4", alpha=0.8)

# 绘制参考航线
_cwp_e = [w[1] for w in circuit_wps]
_cwp_n = [w[0] for w in circuit_wps]
ax1.plot(_cwp_e, _cwp_n, "k--", linewidth=1.0, alpha=0.5, label="参考航线")
for _i, (_ne, _nn) in enumerate(zip(_cwp_e, _cwp_n)):
    ax1.annotate(f"WP{_i+1}", xy=(_ne, _nn), fontsize=9,
                 xytext=(8, 8), textcoords="offset points")
    ax1.scatter(_ne, _nn, color="red", marker="^", s=70, zorder=6)

ax1.set_xlabel("东向 (m)", fontsize=11)
ax1.set_ylabel("北向 (m)", fontsize=11)
ax1.set_title(f"{UAV_NAME} - 航迹对比：ArduPilot vs PX4", fontsize=13)
ax1.set_aspect("equal")
ax1.legend(fontsize=10)
ax1.grid(True, alpha=0.3)
plt.tight_layout()
_save(fig1, "example8_comparison_ground_track")

# ---------------------------------------------------------------------------
# Figure 2: 高度对比
# ---------------------------------------------------------------------------
fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
fig2.suptitle(f"{UAV_NAME} - 高度对比：ArduPilot vs PX4", fontsize=13)

ax2a.plot(t_ap, h_ap["altitude"], color="steelblue", linewidth=1.2, 
          label="ArduPilot", alpha=0.8)
ax2a.plot(t_px4, h_px4["altitude"], color="darkorange", linewidth=1.2, 
          label="PX4", alpha=0.8)
ax2a.axhline(A, color="red", linestyle="--", linewidth=1.0, 
             label=f"目标高度 {A:.0f} m")
ax2a.set_ylabel("高度 (m)", fontsize=11)
ax2a.legend(fontsize=10)
ax2a.grid(True, alpha=0.3)

# 高度误差
alt_err_ap = np.array(h_ap["altitude"]) - A
alt_err_px4 = np.array(h_px4["altitude"]) - A
ax2b.plot(t_ap, alt_err_ap, color="steelblue", linewidth=1.2, 
          label="ArduPilot 误差", alpha=0.8)
ax2b.plot(t_px4, alt_err_px4, color="darkorange", linewidth=1.2, 
          label="PX4 误差", alpha=0.8)
ax2b.axhline(0, color="red", linestyle="--", linewidth=0.8)
ax2b.set_ylabel("高度误差 (m)", fontsize=11)
ax2b.set_xlabel("时间 (s)", fontsize=11)
ax2b.legend(fontsize=10)
ax2b.grid(True, alpha=0.3)

plt.tight_layout()
_save(fig2, "example8_comparison_altitude")

# ---------------------------------------------------------------------------
# Figure 3: 姿态对比
# ---------------------------------------------------------------------------
fig3, axes3 = plt.subplots(2, 2, figsize=(14, 10))
fig3.suptitle(f"{UAV_NAME} - 姿态对比：ArduPilot vs PX4", fontsize=13)

attitude_pairs = [
    ("滚转角 φ (deg)", "phi"),
    ("俯仰角 θ (deg)", "theta"),
    ("滚转角速率 p (deg/s)", "p"),
    ("俯仰角速率 q (deg/s)", "q"),
]

for ax, (title, key) in zip(axes3.flat, attitude_pairs):
    data_ap = np.degrees(h_ap[key]) if key in ["phi", "theta"] else np.degrees(h_ap[key])
    data_px4 = np.degrees(h_px4[key]) if key in ["phi", "theta"] else np.degrees(h_px4[key])
    
    ax.plot(t_ap, data_ap, color="steelblue", linewidth=1.2, 
            label="ArduPilot", alpha=0.8)
    ax.plot(t_px4, data_px4, color="darkorange", linewidth=1.2, 
            label="PX4", alpha=0.8)
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("时间 (s)", fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
_save(fig3, "example8_comparison_attitude")

# ---------------------------------------------------------------------------
# Figure 4: 控制输入对比
# ---------------------------------------------------------------------------
fig4, axes4 = plt.subplots(2, 2, figsize=(14, 10))
fig4.suptitle(f"{UAV_NAME} - 控制输入对比：ArduPilot vs PX4", fontsize=13)

control_pairs = [
    ("升降舵", "elevator"),
    ("副翼", "aileron"),
    ("方向舵", "rudder"),
    ("油门", "throttle"),
]

for ax, (title, key) in zip(axes4.flat, control_pairs):
    ax.plot(t_ap, h_ap[key], color="steelblue", linewidth=1.2, 
            label="ArduPilot", alpha=0.8)
    ax.plot(t_px4, h_px4[key], color="darkorange", linewidth=1.2, 
            label="PX4", alpha=0.8)
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("时间 (s)", fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
_save(fig4, "example8_comparison_controls")

# ---------------------------------------------------------------------------
# Figure 5: 性能指标对比（柱状图）
# ---------------------------------------------------------------------------
fig5, axes5 = plt.subplots(2, 2, figsize=(14, 10))
fig5.suptitle(f"{UAV_NAME} - 性能指标对比：ArduPilot vs PX4", fontsize=13)

metric_groups = [
    ("高度跟踪精度", ["rms_alt_error", "peak_alt_error", "ss_alt_std"], "m"),
    ("姿态平滑度", ["roll_rate_std", "pitch_rate_std"], "deg/s"),
    ("控制平滑度", ["elevator_rate_std", "aileron_rate_std"], "1/s"),
]

# 高度跟踪精度
ax = axes5[0, 0]
metrics_names = ["RMS误差", "峰值误差", "稳态标准差"]
metrics_keys = ["rms_alt_error", "peak_alt_error", "ss_alt_std"]
x = np.arange(len(metrics_names))
width = 0.35
ax.bar(x - width/2, [metrics_ap[k] for k in metrics_keys], width, 
       label="ArduPilot", color="steelblue", alpha=0.8)
ax.bar(x + width/2, [metrics_px4[k] for k in metrics_keys], width, 
       label="PX4", color="darkorange", alpha=0.8)
ax.set_ylabel("高度误差 (m)", fontsize=10)
ax.set_title("高度跟踪精度（越小越好）", fontsize=11)
ax.set_xticks(x)
ax.set_xticklabels(metrics_names, fontsize=9)
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3, axis='y')

# 姿态平滑度
ax = axes5[0, 1]
metrics_names = ["滚转角速率", "俯仰角速率"]
metrics_keys = ["roll_rate_std", "pitch_rate_std"]
x = np.arange(len(metrics_names))
ax.bar(x - width/2, [metrics_ap[k] for k in metrics_keys], width, 
       label="ArduPilot", color="steelblue", alpha=0.8)
ax.bar(x + width/2, [metrics_px4[k] for k in metrics_keys], width, 
       label="PX4", color="darkorange", alpha=0.8)
ax.set_ylabel("标准差 (deg/s)", fontsize=10)
ax.set_title("姿态变化率标准差（越小越平滑）", fontsize=11)
ax.set_xticks(x)
ax.set_xticklabels(metrics_names, fontsize=9)
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3, axis='y')

# 控制平滑度
ax = axes5[1, 0]
metrics_names = ["升降舵变化率", "副翼变化率"]
metrics_keys = ["elevator_rate_std", "aileron_rate_std"]
x = np.arange(len(metrics_names))
ax.bar(x - width/2, [metrics_ap[k] for k in metrics_keys], width, 
       label="ArduPilot", color="steelblue", alpha=0.8)
ax.bar(x + width/2, [metrics_px4[k] for k in metrics_keys], width, 
       label="PX4", color="darkorange", alpha=0.8)
ax.set_ylabel("标准差 (1/s)", fontsize=10)
ax.set_title("控制输入变化率标准差（越小越平滑）", fontsize=11)
ax.set_xticks(x)
ax.set_xticklabels(metrics_names, fontsize=9)
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3, axis='y')

# 综合对比表格
ax = axes5[1, 1]
ax.axis('off')
table_data = []
table_data.append(["指标", "ArduPilot", "PX4", "差异"])
table_data.append(["RMS高度误差 (m)", 
                   f"{metrics_ap['rms_alt_error']:.3f}",
                   f"{metrics_px4['rms_alt_error']:.3f}",
                   f"{metrics_ap['rms_alt_error'] - metrics_px4['rms_alt_error']:+.3f}"])
table_data.append(["稳态高度标准差 (m)", 
                   f"{metrics_ap['ss_alt_std']:.3f}",
                   f"{metrics_px4['ss_alt_std']:.3f}",
                   f"{metrics_ap['ss_alt_std'] - metrics_px4['ss_alt_std']:+.3f}"])
table_data.append(["滚转平滑度 (deg/s)", 
                   f"{metrics_ap['roll_rate_std']:.3f}",
                   f"{metrics_px4['roll_rate_std']:.3f}",
                   f"{metrics_ap['roll_rate_std'] - metrics_px4['roll_rate_std']:+.3f}"])
table_data.append(["俯仰平滑度 (deg/s)", 
                   f"{metrics_ap['pitch_rate_std']:.3f}",
                   f"{metrics_px4['pitch_rate_std']:.3f}",
                   f"{metrics_ap['pitch_rate_std'] - metrics_px4['pitch_rate_std']:+.3f}"])

table = ax.table(cellText=table_data, cellLoc='center', loc='center',
                 colWidths=[0.35, 0.22, 0.22, 0.21])
table.auto_set_font_size(False)
table.set_fontsize(9)
table.scale(1, 2)
# 表头加粗
for i in range(4):
    table[(0, i)].set_facecolor('#E0E0E0')
    table[(0, i)].set_text_props(weight='bold')

ax.set_title("性能对比总结", fontsize=11, pad=20)

plt.tight_layout()
_save(fig5, "example8_comparison_metrics")

# ---------------------------------------------------------------------------
# 打印对比总结
# ---------------------------------------------------------------------------
print("\n" + "=" * 70)
print("性能对比总结")
print("=" * 70)
print(f"\n{'指标':<25} {'ArduPilot':>12} {'PX4':>12} {'差异':>12}")
print("-" * 70)
for key in metrics_ap.keys():
    diff = metrics_ap[key] - metrics_px4[key]
    print(f"{key:<25} {metrics_ap[key]:>12.4f} {metrics_px4[key]:>12.4f} {diff:>+12.4f}")

print("\n" + "=" * 70)
print("结论:")
print("-" * 70)

# 判断哪个控制器更好
if metrics_ap['rms_alt_error'] < metrics_px4['rms_alt_error']:
    print("✓ ArduPilot 在高度跟踪精度上表现更好")
else:
    print("✓ PX4 在高度跟踪精度上表现更好")

if metrics_ap['roll_rate_std'] < metrics_px4['roll_rate_std']:
    print("✓ ArduPilot 在滚转平滑度上表现更好")
else:
    print("✓ PX4 在滚转平滑度上表现更好")

if metrics_ap['elevator_rate_std'] < metrics_px4['elevator_rate_std']:
    print("✓ ArduPilot 在控制平滑度上表现更好")
else:
    print("✓ PX4 在控制平滑度上表现更好")

print("\n注意：性能差异取决于具体的参数调优。")
print("建议针对每种控制律进行独立调参以达到最优性能。")
print("=" * 70)

print(f"\n完成！所有输出文件保存在: {os.path.abspath(OUTPUT_DIR)}")
