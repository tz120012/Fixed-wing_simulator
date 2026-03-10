"""计算配平状态下的巡航油门."""
import sys; sys.path.insert(0, 'src')
from models.aircraft_database import get_aircraft_params
from dynamics.nonlinear_model import NonlinearModel, Controls
from dynamics.aerodynamics import compute_aero_forces
import numpy as np

params = get_aircraft_params('TB2')
dyn = NonlinearModel(params)
trim = dyn.compute_trim()
p = params  # already a dict
m = p['mass']
G = 9.80665
T_max = m * G * 0.20

print(f"Aircraft mass : {m:.1f} kg")
print(f"Trim U0       : {trim.U0:.2f} m/s")
print(f"Trim alpha    : {np.degrees(trim.alpha_trim):.3f} deg")
print(f"T_max (TWR=0.20) : {T_max:.1f} N")

# Compute aero forces at trim
u0 = trim.U0 * np.cos(trim.alpha_trim)
w0 = trim.U0 * np.sin(trim.alpha_trim)
# 4 independent surfaces: da_left=0, da_right=0, dv_left=de_trim, dv_right=de_trim
aero = compute_aero_forces(u0, 0, w0, 0, 0, 0, 0, 0, trim.de_trim, trim.de_trim, 0.5, p)

# Gravity in body frame at trim (phi=0, theta=alpha_trim)
alpha = trim.alpha_trim
Xg = -m * G * np.sin(alpha)
Zg =  m * G * np.cos(alpha)

# For level flight: X_tot = 0 → Thrust = -X_aero - Xg
Thrust_needed = -aero.X - Xg
thr_cruise = Thrust_needed / T_max

print(f"Aero X force  : {aero.X:.1f} N")
print(f"Gravity X     : {Xg:.1f} N")
print(f"Thrust needed : {Thrust_needed:.1f} N")
print(f"thr_cruise    : {thr_cruise:.4f}")

# Also verify Z balance
Z_total = aero.Z + Zg + 0  # throttle has no Z component in simple model
print(f"Z balance (should be ~0): {Z_total:.1f} N  (m*az = {m*9.8:.1f})")
