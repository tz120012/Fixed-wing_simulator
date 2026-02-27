"""
example_3_trajectory_tracking.py
===================================
Closed-loop AUTO mode: TB2 tracking a minimum-snap square trajectory.

Demonstrates:
  - FixedWingSimulator in AUTO mode
  - WaypointManager with minimum_snap trajectory
  - 5-layer ArduPilot control chain
  - 3D trajectory visualisation
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from simulation.simulator import FixedWingSimulator

# ---- Create simulator ------------------------------------------------------
sim = FixedWingSimulator(
    aircraft_name="TB2",
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
print("Running closed-loop trajectory tracking...")
result = sim.run(closed_loop=True)
print(result.summary())

# ---- Visualise -------------------------------------------------------------
result.visualize()
