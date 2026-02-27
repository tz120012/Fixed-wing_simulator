"""诊断 segment.end 传入值."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import matplotlib
matplotlib.use('Agg')
import numpy as np

from simulation.simulator import FixedWingSimulator
from control.navigation_controller import NavigationController, PathSegment

# monkey-patch 记录 segment.end
_orig_update = NavigationController.update
_segs = []

def _debug_update(self, state, segment, dt=0.1):
    _segs.append((-segment.end[2], state.altitude))   # (target_alt, current_alt)
    return _orig_update(self, state, segment, dt)

NavigationController.update = _debug_update

sim = FixedWingSimulator(
    aircraft_name='TB2', dt=0.01, duration=5.0,
    initial_mode='FBW_B', wind_type='NONE'
)
sim.wp_mgr.add_waypoint(0.0, 0.0,  0.0)
sim.wp_mgr.add_waypoint(0.0, 0.0, 80.0)
result = sim.run(closed_loop=True)

segs = np.array(_segs)
print("t  |  seg_end_alt  |  cur_alt")
for i in range(0, len(segs), 100):
    print(f"{i*0.01:5.1f}s  seg_end={segs[i,0]:.1f}m  cur_alt={segs[i,1]:.1f}m")

print(f"\nMax seg_end_alt: {segs[:,0].max():.1f}m")
print(f"Min seg_end_alt: {segs[:,0].min():.1f}m")

# 检查 des_pos 在 simulator 里
print("\n-- Checking raw des_pos before clamp --")
from planning.waypoint_manager import WaypointManager
sim2 = FixedWingSimulator(
    aircraft_name='TB2', dt=0.01, duration=5.0,
    initial_mode='FBW_B', wind_type='NONE'
)
sim2.wp_mgr.add_waypoint(0.0, 0.0,  0.0)
sim2.wp_mgr.add_waypoint(0.0, 0.0, 80.0)
traj = sim2.wp_mgr.trajectory
wps = np.array(sim2.wp_mgr._waypoints_ned)
print(f"Waypoints NED: {wps}")
print(f"alt_min={float(np.min(-wps[:,2])):.1f}m  alt_max={float(np.max(-wps[:,2])):.1f}m")

for t_sample in [0.0, 0.5, 1.0, 1.5, 2.0]:
    ds = traj.desired_state(t_sample)
    print(f"t={t_sample:.1f}s  des.pos={ds.pos}  alt={-ds.pos[2]:.1f}m")
