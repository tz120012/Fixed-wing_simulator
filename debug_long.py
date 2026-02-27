"""长时间测试 TECS 收敛性."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import matplotlib
matplotlib.use('Agg')
import numpy as np

from simulation.simulator import FixedWingSimulator
from control.navigation_controller import NavigationController

_orig_update = NavigationController.update
_log = dict(t=[], alt=[], spd=[], thr=[], pitch=[])
_step = [0]

def _debug_update(self, state, segment, dt=0.1):
    result = _orig_update(self, state, segment, dt)
    _log['alt'].append(state.altitude)
    _log['thr'].append(self.tecs.output.throttle_dem)
    _log['pitch'].append(np.degrees(self.tecs.output.pitch_dem))
    _log['spd'].append(state.airspeed)
    _step[0] += 1
    return result

NavigationController.update = _debug_update

sim = FixedWingSimulator(
    aircraft_name='TB2', dt=0.01, duration=60.0,
    initial_mode='FBW_B', wind_type='NONE'
)
sim.wp_mgr.add_waypoint(0.0, 0.0,  0.0)
sim.wp_mgr.add_waypoint(0.0, 0.0, 80.0)
result = sim.run(closed_loop=True)

n = len(_log['thr'])
t_arr = np.arange(n) * 0.01
alt_arr = np.array(_log['alt'])
spd_arr = np.array(_log['spd'])

print(f"{'t':>5s} | {'altitude':>8s} | {'speed':>7s} | {'thr':>6s} | {'pitch°':>7s}")
for i in range(0, n, 500):
    print(f"{t_arr[i]:5.1f}s | {alt_arr[i]:8.1f} | {spd_arr[i]:7.2f} | {_log['thr'][i]:6.3f} | {_log['pitch'][i]:7.2f}")

print(f"\nMax altitude   : {alt_arr.max():.2f} m  at t={t_arr[alt_arr.argmax()]:.1f}s")
print(f"Min altitude   : {alt_arr.min():.2f} m  at t={t_arr[alt_arr.argmin()]:.1f}s")
print(f"Final altitude : {alt_arr[-1]:.2f} m  (target 80 m)")
print(f"Final speed    : {spd_arr[-1]:.2f} m/s  (cruise 40 m/s)")

# Check final 10s stability
final_alt = alt_arr[-1000:]
final_spd = spd_arr[-1000:]
print(f"\nFinal 10s stats:")
print(f"  Alt: mean={final_alt.mean():.2f}  std={final_alt.std():.2f}  range=[{final_alt.min():.1f}, {final_alt.max():.1f}]")
print(f"  Spd: mean={final_spd.mean():.2f}  std={final_spd.std():.2f}  range=[{final_spd.min():.1f}, {final_spd.max():.1f}]")
