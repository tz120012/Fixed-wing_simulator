"""
linear_model.py  –  4-DOF longitudinal linearized state-space model.

State vector: x = [u_p, alpha, q, theta]
  u_p   : forward-speed perturbation (normalised, u_p = Δu / U0)
  alpha : angle of attack perturbation (rad)
  q     : pitch rate (rad/s)
  theta : pitch angle perturbation (rad)

Control input: u_c = [delta_T, delta_e]
  delta_T : throttle perturbation (dimensionless)
  delta_e : elevator deflection (rad)

Reference:
  Stevens & Lewis, "Aircraft Control and Simulation", 3rd Ed., Ch. 3
"""

import numpy as np
from typing import Dict, Any, List, Tuple
from scipy.integrate import solve_ivp

# Physical constants (replicate project-1 constants.py values)
G         = 9.80665
RHO0      = 1.225
R_GAS     = 287.05
GAMMA     = 1.4
A_SOUND   = np.sqrt(GAMMA * R_GAS * 288.15)  # ≈ 340.3 m/s


class ModeResult:
    """Result of a single eigenvalue modal analysis."""

    def __init__(self, name: str, eigenvalue: complex,
                 wn: float, zeta: float, stable: bool):
        self.name       = name
        self.eigenvalue = eigenvalue
        self.wn         = wn   # natural frequency (rad/s)
        self.zeta       = zeta # damping ratio
        self.stable     = stable

    def __str__(self) -> str:
        s  = f"{self.name}: λ = {self.eigenvalue:.4f} | "
        s += f"ωn = {self.wn:.4f} rad/s | ζ = {self.zeta:.3f}"
        if self.zeta < 0:
            s += "  → UNSTABLE"
        elif self.zeta < 0.02:
            s += "  → Very poorly damped"
        elif self.zeta < 0.2:
            s += "  → Poorly damped"
        elif self.zeta < 0.7:
            s += "  → Good aircraft mode"
        else:
            s += "  → Highly damped"
        return s


class LinearAnalysisResult:
    """Complete result from a 4-DOF linear simulation."""

    def __init__(self, t: np.ndarray, y: np.ndarray, de: np.ndarray,
                 U0: float, modes: List[ModeResult], A: np.ndarray,
                 B: np.ndarray, uav_name: str):
        self.t        = t
        self.y        = y        # (4, N) state history
        self.de       = de       # (N,) elevator input history (rad)
        self.U0       = U0       # trim speed (m/s)
        self.modes    = modes
        self.A        = A
        self.B        = B
        self.uav_name = uav_name

    def summary(self) -> str:
        lines = [
            f"=== {self.uav_name} 4-DOF Linear Analysis ===",
            f"Trim speed U0 = {self.U0:.2f} m/s ({self.U0 * 1.94384:.2f} kn)",
            "",
            "Flight Dynamics Modes (Eigenvalues of A):",
        ]
        lines += [f"  {m}" for m in self.modes]
        return "\n".join(lines)

    def plot(self):
        """Quick Matplotlib plot of time-domain response (standalone use)."""
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 2, figsize=(12, 10))
        fig.suptitle(f"{self.uav_name} 4-DOF Time Domain Response", fontsize=14)

        labels = [
            ("u_p × U0 (m/s)", self.y[0] * self.U0),
            ("α (deg)",        np.degrees(self.y[1])),
            ("q (deg/s)",      np.degrees(self.y[2])),
            ("θ (deg)",        np.degrees(self.y[3])),
            ("Elevator (deg)", np.degrees(self.de)),
        ]
        for ax, (lbl, data) in zip(axes.flat, labels):
            ax.plot(self.t, data)
            ax.set_xlabel("Time (s)")
            ax.set_title(lbl)
            ax.grid(True, alpha=0.3)

        axes.flat[-1].set_visible(False)
        plt.tight_layout()
        plt.show()


class LinearModel:
    """
    4-DOF longitudinal linearized state-space model for a fixed-wing aircraft.

    Compatible with project-1 FlightSimState._build_state_space() output.
    """

    def __init__(self, params: Dict[str, Any]):
        """
        Parameters
        ----------
        params : aircraft parameter dict (UAVParameter format from aircraft_database)
        """
        self.params = params
        self._A: np.ndarray | None = None
        self._B: np.ndarray | None = None
        self._U0: float = 0.0

    # ------------------------------------------------------------------
    # Build state-space
    # ------------------------------------------------------------------

    def build(self) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Construct A and B matrices for the 4-DOF longitudinal model.

        Returns
        -------
        A  : (4, 4) state matrix
        B  : (4, 2) input matrix  [delta_T, delta_e]
        U0 : trim airspeed (m/s)
        """
        p = self.params
        m, S, c_bar, Iyy, mach_val = (
            p["mass"], p["S"], p["c"], p["Iyy"], p["Mach"]
        )
        U0  = mach_val * A_SOUND
        rho = RHO0

        # Non-dimensional mass / inertia coefficients
        m1  = m / (0.5 * rho * U0 * S)
        c1  = c_bar / (2.0 * U0)
        Jy1 = Iyy / (0.5 * rho * U0**2 * S * c_bar)

        CL0, CLa, CLq, CLde, CLu = (
            p["CL_0"], p["CL_alpha"], p["CL_q"], p["CL_deltae"], p["CL_u"]
        )
        CD0, CDa, CDq, CDde, CDu = (
            p["CD_0"], p["CD_alpha"], p["CD_q"], p["CD_deltae"], p["CD_u"]
        )
        Cm0, Cma, Cmq, Cmde, Cmu = (
            p["Cm_0"], p["Cm_alpha"], p["Cm_q"], p["Cm_deltae"], p["Cm_u"]
        )

        # Stability derivatives in body axis
        CXu = -2.0 * CD0 - CDu
        CXa = -CDa + CL0
        CXq = -CDq
        CZu = -2.0 * CL0 - CLu
        CZa = -CLa - CD0
        CZq = -CLq
        Cmu_val  = 2.0 * Cm0 + Cmu
        Cma_val  = Cma + Cm0
        Cmq_val  = Cmq
        CXde     = -CDde
        CZde     = -CLde
        Cmde_val = Cmde

        M = np.array([
            [m1,   0,    0,    0],
            [0,    m1,   0,    0],
            [0,    0,    Jy1,  0],
            [0,    0,    0,    1],
        ])
        K = np.array([
            [-CXu, -CXa, -CXq,            m1 * (G / U0)],
            [-CZu, -CZa, -c1 * CZq - m1,  0            ],
            [-Cmu_val, -Cma_val, -c1 * Cmq_val, 0       ],
            [0,    0,   -1,               0             ],
        ])
        B_raw = np.array([
            [0, CXde ],
            [0, CZde ],
            [0, Cmde_val],
            [0, 0   ],
        ])

        A = np.linalg.solve(-M, K)
        B = np.linalg.solve(M, B_raw)

        self._A  = A
        self._B  = B
        self._U0 = U0
        return A, B, U0

    # ------------------------------------------------------------------
    # Modal analysis
    # ------------------------------------------------------------------

    def analyze_modes(self, A: np.ndarray = None) -> List[ModeResult]:
        """
        Eigenvalue decomposition of A to identify longitudinal modes.

        Returns a list of ModeResult (Short Period, Phugoid, Subsidence).
        """
        if A is None:
            if self._A is None:
                self.build()
            A = self._A

        eigvals, _ = np.linalg.eig(A)

        # --- collect (eigenvalue, wn, zeta) tuples ----------------------
        mode_data = []
        for lv in eigvals:
            wn    = abs(lv)
            sigma = lv.real
            zeta  = (-sigma / wn) if wn > 1e-9 else 0.0
            mode_data.append((lv, wn, zeta))

        mode_data.sort(key=lambda x: x[1], reverse=True)

        processed = set()
        results: List[ModeResult] = []

        for i, (lv, wn, zeta) in enumerate(mode_data):
            if i in processed:
                continue
            is_complex = abs(lv.imag) > 1e-5

            if is_complex:
                # Find conjugate pair
                for j, (other, _, _) in enumerate(mode_data):
                    if (i != j
                            and abs(lv.real - other.real) < 1e-5
                            and abs(lv.imag + other.imag) < 1e-5):
                        processed.add(i)
                        processed.add(j)
                        name = "Short Period Mode" if wn > 0.5 else "Phugoid Mode"
                        results.append(ModeResult(name, lv, wn, zeta, lv.real < 0))
                        break
            else:
                processed.add(i)
                results.append(ModeResult("Subsidence Mode", lv, wn, zeta, lv.real < 0))

        return results

    # ------------------------------------------------------------------
    # Time-domain simulation
    # ------------------------------------------------------------------

    def simulate(
        self,
        pulses: List[Dict],
        duration: float = 10.0,
        n_points: int = 500,
        A: np.ndarray = None,
        B: np.ndarray = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simulate 4-DOF longitudinal response to elevator pulse inputs.

        Parameters
        ----------
        pulses   : list of pulse dicts with keys:
                     start_time (s), duration (s), angle_deg (deg)
        duration : simulation duration (s)
        n_points : number of time evaluation points
        A, B     : optional override matrices; uses cached build() output otherwise

        Returns
        -------
        t  : (N,) time array (s)
        y  : (4, N) state history  [u_p, alpha, q, theta]
        de : (N,) elevator input history (rad)
        """
        if A is None or B is None:
            if self._A is None:
                self.build()
            A = self._A
            B = self._B

        t_eval = np.linspace(0, duration, n_points)

        def delta_e(t_val: float) -> float:
            for p in pulses:
                if p["start_time"] <= t_val <= p["start_time"] + p["duration"]:
                    return p["angle_deg"] * np.pi / 180.0
            return 0.0

        def f_ode(t_val, y_val):
            u_in = np.array([0.0, delta_e(t_val)])
            return A @ y_val + B @ u_in

        y0  = np.zeros(4)
        sol = solve_ivp(f_ode, [0, duration], y0,
                        t_eval=t_eval, rtol=1e-6, atol=1e-6)

        de_arr = np.array([delta_e(tt) for tt in sol.t])
        return sol.t, sol.y, de_arr

    # ------------------------------------------------------------------
    # Full analysis pipeline (convenience wrapper)
    # ------------------------------------------------------------------

    def run_analysis(self, pulses: List[Dict], duration: float = 10.0,
                     uav_name: str = "UAV") -> "LinearAnalysisResult":
        """Build, analyse modes, simulate and return a LinearAnalysisResult."""
        A, B, U0  = self.build()
        modes     = self.analyze_modes(A)
        t, y, de  = self.simulate(pulses, duration, A=A, B=B)
        return LinearAnalysisResult(t, y, de, U0, modes, A, B, uav_name)
