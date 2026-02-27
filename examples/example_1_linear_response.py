"""
example_1_linear_response.py
==============================
4-DOF linear model pulse response – backward-compatible with project-1.

Demonstrates:
  - LinearModel.run_analysis()
  - Modal analysis (Short Period, Phugoid, Subsidence)
  - Time-domain response to elevator pulse
  - Results match project-1 FlightSimState results exactly
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from models.aircraft_database import get_aircraft_params
from dynamics.linear_model    import LinearModel

# ---- Select aircraft -------------------------------------------------------
UAV_NAME = "TB2"
params   = get_aircraft_params(UAV_NAME)

# ---- Build linear model and run analysis ----------------------------------
model  = LinearModel(params)
pulses = [{"start_time": 1.0, "duration": 0.5, "angle_deg": 2.0}]

result = model.run_analysis(pulses, duration=10.0, uav_name=UAV_NAME)

# ---- Print results ---------------------------------------------------------
print(result.summary())
print()
for mode in result.modes:
    print(f"  {mode}")

# ---- Plot ------------------------------------------------------------------
result.plot()
