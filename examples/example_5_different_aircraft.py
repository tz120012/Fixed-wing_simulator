"""
example_5_different_aircraft.py
==================================
Compare all 7 aircraft in the database under the same elevator pulse.

Demonstrates:
  - Aircraft parameter database (all 7 UAVs)
  - Parallel 4-DOF linear responses
  - Short-period and phugoid modal comparison
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
import matplotlib.pyplot as plt

from models.aircraft_database import AIRCRAFT_NAMES, get_aircraft_params
from dynamics.linear_model    import LinearModel

pulses   = [{"start_time": 1.0, "duration": 0.5, "angle_deg": 2.0}]
duration = 15.0

results = {}
for name in AIRCRAFT_NAMES:
    params   = get_aircraft_params(name)
    model    = LinearModel(params)
    result   = model.run_analysis(pulses, duration=duration, uav_name=name)
    results[name] = result
    print(f"{name:12s}  U0={result.U0:6.1f} m/s  |  ", end="")
    for m in result.modes:
        print(f"{m.name[:4]}: ζ={m.zeta:.3f}", end="  ")
    print()

# ---- Compare pitch-rate responses ------------------------------------------
fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
fig.suptitle("All Aircraft – 4-DOF Pitch Rate Response to 2-deg Elevator Pulse")

colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

for i, (name, res) in enumerate(results.items()):
    c = colors[i % len(colors)]
    axes[0].plot(res.t, np.degrees(res.y[2]), label=name, color=c, linewidth=1.2)
    axes[1].plot(res.t, np.degrees(res.y[3]), label=name, color=c, linewidth=1.2)

axes[0].set_ylabel("q (deg/s)")
axes[0].legend(fontsize=8, ncol=4)
axes[0].grid(True, alpha=0.3)
axes[1].set_ylabel("θ (deg)")
axes[1].set_xlabel("Time (s)")
axes[1].grid(True, alpha=0.3)

# ---- Short-period comparison table -----------------------------------------
print("\nShort-Period Mode Comparison:")
print(f"{'Aircraft':12s}  {'U0 (m/s)':>9}  {'wn (rad/s)':>11}  {'zeta':>7}  {'Stable':>7}")
print("-" * 55)
for name, res in results.items():
    for m in res.modes:
        if "Short Period" in m.name:
            print(f"{name:12s}  {res.U0:9.2f}  {m.wn:11.4f}  {m.zeta:7.3f}  "
                  f"{'YES' if m.stable else 'NO!':>7}")

plt.tight_layout()
plt.show()
