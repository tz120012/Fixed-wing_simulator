"""
example_2_nonlinear_dynamics.py
=================================
6-DOF nonlinear dynamics open-loop simulation.

Demonstrates:
  - NonlinearModel.compute_trim()  – level-flight trim computation
  - NonlinearModel.simulate()      – free-response to aileron pulse
  - Full 12-state trajectory (position, velocity, attitude, rates)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
from models.aircraft_database import get_aircraft_params
from dynamics.nonlinear_model import NonlinearModel

# ---- Aircraft setup --------------------------------------------------------
UAV_NAME = "Predator"
params   = get_aircraft_params(UAV_NAME)
model    = NonlinearModel(params)

# ---- Trim condition --------------------------------------------------------
trim = model.compute_trim()
print(f"Trim speed  : {trim.U0:.2f} m/s ({trim.U0 * 1.944:.2f} kn)")
print(f"Trim alpha  : {np.degrees(trim.alpha_trim):.3f} deg")
print(f"Trim delta_e: {np.degrees(trim.de_trim):.3f} deg")

# ---- Simulate: 5-deg aileron pulse at t=2s --------------------------------
pulses = [
    {"start_time": 2.0, "duration": 0.5, "roll_deg": 5.0,
     "angle_deg": 0.0, "yaw_deg": 0.0, "throttle": 1.0},
]

result = model.simulate(pulses, duration=15.0, n_points=1000)
print(result.summary())

# ---- Plot ------------------------------------------------------------------
result.plot()
