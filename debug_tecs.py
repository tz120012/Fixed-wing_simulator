"""TECS 诊断脚本 - 分析 example_1 的高度控制行为."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import matplotlib
matplotlib.use('Agg')
import numpy as np

from simulation.simulator import FixedWingSimulator
from control.navigation_controller import NavigationController

# --- monkey-patch 添加详细日志 ---
_orig_update = NavigationController.update
_log = dict(t=[], hgt_raw=[], hgt_lpf=[], alt=[], thr=[], pitch=[], spd=[], ste=[])
_step = [0]

def _debug_update(self, state, segment, dt=0.1):
    result = _orig_update(self, state, segment, dt)
    _log['hgt_raw'].append(self.tecs._hgt_dem_raw)
    _log['hgt_lpf'].append(self.tecs._hgt_dem)
    _log['alt'].append(state.altitude)
    _log['thr'].append(self.tecs.output.throttle_dem)
    _log['pitch'].append(np.degrees(self.tecs.output.pitch_dem))
    _log['spd'].append(state.airspeed)
    _log['ste'].append(self.tecs._STE_error)
    _step[0] += 1
    return result

NavigationController.update = _debug_update

sim = FixedWingSimulator(
    aircraft_name='TB2', dt=0.01, duration=20.0,
    initial_mode='FBW_B', wind_type='NONE'
)
sim.wp_mgr.add_waypoint(0.0, 0.0,  0.0)
sim.wp_mgr.add_waypoint(0.0, 0.0, 80.0)
result = sim.run(closed_loop=True)
h = result.history.to_dict()

n = len(_log['thr'])
t_arr = np.arange(n) * 0.01

print("="*85)
print(f"{'t':>5s} | {'hgt_raw':>8s} | {'hgt_lpf':>8s} | {'altitude':>8s} | {'thr':>6s} | {'pitch°':>7s} | {'speed':>6s} | {'STE_err':>8s}")
print("="*85)
for i in range(0, n, 100):
    print(f"{t_arr[i]:5.1f}s | {_log['hgt_raw'][i]:8.1f} | {_log['hgt_lpf'][i]:8.1f} | {_log['alt'][i]:8.1f} | "
          f"{_log['thr'][i]:6.3f} | {_log['pitch'][i]:7.2f} | {_log['spd'][i]:6.2f} | {_log['ste'][i]:8.1f}")

alt_arr = np.array(h['altitude'])
spd_arr = np.array(h['airspeed'])
t_full  = np.array(h['t'])

print()
print(f"Final altitude : {alt_arr[-1]:.2f} m  (target 80 m)")
print(f"Max altitude   : {alt_arr.max():.2f} m  at t={t_full[alt_arr.argmax()]:.1f}s")
print(f"Final airspeed : {spd_arr[-1]:.2f} m/s  (cruise 40 m/s)")
print(f"Max airspeed   : {spd_arr.max():.2f} m/s")

# 寻找首次超过 78m 的时间
above78 = np.where(alt_arr >= 78)[0]
if len(above78):
    print(f"Reached 78m    : t={t_full[above78[0]]:.2f}s")
above80 = np.where(alt_arr >= 80)[0]
if len(above80):
    print(f"Reached 80m    : t={t_full[above80[0]]:.2f}s  (first overshoot)")
