"""
example_4_wind_resistance.py
==============================
FBW_B mode flight under random sinusoidal wind disturbance.

Demonstrates:
  - Wind model (RANDOMSINE)
  - FBW_B flight mode (altitude + speed hold)
  - Disturbance rejection
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from simulation.simulator     import FixedWingSimulator
from visualization.plotter    import FixedWingPlotter
import matplotlib.pyplot as plt
import numpy as np

# ---- Create simulator with wind --------------------------------------------
sim = FixedWingSimulator(
    aircraft_name="Anka",
    dt=0.01,
    duration=30.0,
    initial_mode="FBW_B",
    wind_type="RANDOMSINE",
    traj_type="minimum_snap",
)

# Single waypoint far ahead to keep the aircraft flying straight
sim.wp_mgr.add_waypoint(north=2000.0, east=0.0, alt_m=200.0)
sim.wp_mgr.add_waypoint(north=4000.0, east=0.0, alt_m=200.0)

print("Running FBW_B + RANDOMSINE wind simulation...")
result = sim.run(closed_loop=True)
print(result.summary())

# ---- Compare altitude deviation --------------------------------------------
h = result.history.to_dict()
t   = h["t"]
alt = h["altitude"]
spd = h["airspeed"]

fig, axes = plt.subplots(1, 2, figsize=(12, 4))
axes[0].plot(t, alt);  axes[0].set_title("Altitude (m)"); axes[0].grid(True, alpha=0.3)
axes[0].axhline(200, color='r', linestyle='--', label='Reference')
axes[0].legend()
axes[1].plot(t, spd, color='g'); axes[1].set_title("Airspeed (m/s)"); axes[1].grid(True, alpha=0.3)
fig.suptitle("Anka – FBW_B mode under RANDOMSINE wind")
plt.tight_layout()
plt.show()
