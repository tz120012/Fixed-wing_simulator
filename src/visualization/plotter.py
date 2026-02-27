"""
plotter.py  –  2D time-domain plots for fixed-wing simulation results.

Provides both:
  - Matplotlib static figures (for standalone scripts, project-2 style)
  - Plotly figures (for Reflex web UI, project-1 style)
"""

from __future__ import annotations

import numpy as np
from typing import Dict, Optional


# ---------------------------------------------------------------------------
# Plotly-based plotter (compatible with Reflex web UI)
# ---------------------------------------------------------------------------

class FixedWingPlotter:
    """Create Plotly figures from simulation history dicts."""

    @staticmethod
    def plot_4dof(
        t: np.ndarray,
        y: np.ndarray,
        de: np.ndarray,
        U0: float,
        uav_name: str = "UAV",
    ):
        """
        Plot 4-DOF longitudinal state + elevator input.

        Backward-compatible with project-1's _create_time_domain_plot().

        Returns
        -------
        plotly.graph_objects.Figure
        """
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots

        fig = make_subplots(
            rows=3, cols=2,
            subplot_titles=(
                "u_p × U0 (m/s): Forward speed perturbation",
                "α (deg): Angle of attack",
                "q (deg/s): Pitch rate",
                "θ (deg): Pitch angle",
                "Elevator (deg): Input",
                "",
            ),
            vertical_spacing=0.12,
        )
        fig.add_trace(go.Scatter(x=t, y=y[0]*U0, name="u_p"),  row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=np.degrees(y[1]), name="α"),  row=1, col=2)
        fig.add_trace(go.Scatter(x=t, y=np.degrees(y[2]), name="q"),  row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=np.degrees(y[3]), name="θ"),  row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=np.degrees(de),   name="δe"), row=3, col=1)

        fig.update_layout(
            title_text=f"{uav_name} 4-DOF Time Domain Response",
            height=700, showlegend=True,
        )
        fig.update_xaxes(title_text="Time (s)")
        return fig

    @staticmethod
    def plot_6dof(history: Dict[str, np.ndarray], uav_name: str = "UAV"):
        """
        Plot full 6-DOF simulation history.

        Returns
        -------
        plotly.graph_objects.Figure
        """
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots

        t = history["t"]
        rows_data = [
            ("u (m/s)",       history["u"]),
            ("v (m/s)",       history["v"]),
            ("w (m/s)",       history["w"]),
            ("p (deg/s)",     np.degrees(history["p"])),
            ("q (deg/s)",     np.degrees(history["q"])),
            ("r (deg/s)",     np.degrees(history["r"])),
            ("φ (deg)",       np.degrees(history["phi"])),
            ("θ (deg)",       np.degrees(history["theta"])),
            ("ψ (deg)",       np.degrees(history["psi"])),
            ("Elevator",      history["elevator"]),
            ("Aileron",       history["aileron"]),
            ("Rudder",        history["rudder"]),
            ("Throttle",      history["throttle"]),
            ("α (deg)",       np.degrees(history["alpha"])),
            ("Airspeed (m/s)",history["airspeed"]),
            ("Altitude (m)",  history["altitude"]),
        ]

        n = len(rows_data)
        fig = make_subplots(rows=n, cols=1,
                             subplot_titles=[r[0] for r in rows_data],
                             vertical_spacing=0.02)
        for i, (lbl, data) in enumerate(rows_data, start=1):
            fig.add_trace(go.Scatter(x=t, y=data, name=lbl, showlegend=False),
                          row=i, col=1)
        fig.update_layout(
            title_text=f"{uav_name} 6-DOF Response",
            height=300 * n,
        )
        fig.update_xaxes(title_text="Time (s)")
        return fig

    @staticmethod
    def plot_3d_trajectory(history: Dict[str, np.ndarray], uav_name: str = "UAV"):
        """3D NED trajectory plot (Plotly)."""
        import plotly.graph_objects as go

        x = history["x_east"]   # E axis → X in plot
        y = history["x_north"]  # N axis → Y in plot
        z = history["altitude"] # altitude up

        traces = [
            go.Scatter3d(
                x=x, y=y, z=z,
                mode="lines", name="Actual",
                line=dict(color="blue", width=3),
            ),
            go.Scatter3d(
                x=[x[0]], y=[y[0]], z=[z[0]],
                mode="markers", name="Start",
                marker=dict(color="green", size=6),
            ),
        ]
        # Desired trajectory (if available)
        if "des_north" in history and not np.all(history["des_north"] == 0):
            traces.append(go.Scatter3d(
                x=history["des_east"],
                y=history["des_north"],
                z=-history["des_down"],
                mode="lines", name="Desired",
                line=dict(color="red", dash="dash", width=2),
            ))

        fig = go.Figure(data=traces)
        fig.update_layout(
            title=f"{uav_name} 3D Trajectory",
            height=700,
            scene=dict(
                xaxis_title="East (m)",
                yaxis_title="North (m)",
                zaxis_title="Altitude (m)",
            ),
        )
        return fig


# ---------------------------------------------------------------------------
# Matplotlib-based plotter (standalone / project-2 style)
# ---------------------------------------------------------------------------

    @staticmethod
    def plot_6dof_matplotlib(history: Dict[str, np.ndarray], uav_name: str = "UAV") -> None:
        """
        Create static Matplotlib figures (8 subplots, same style as project-2 display.py).
        """
        import matplotlib.pyplot as plt

        t = history["t"]
        deg = np.degrees

        fig1, axes = plt.subplots(2, 3, figsize=(15, 8))
        fig1.suptitle(f"{uav_name} – Position & Velocity", fontsize=13)
        _pairs = [
            ("North (m)",  history["x_north"]),
            ("East  (m)",  history["x_east"]),
            ("Alt   (m)",  history["altitude"]),
            ("u (m/s)",    history["u"]),
            ("v (m/s)",    history["v"]),
            ("w (m/s)",    history["w"]),
        ]
        for ax, (lbl, data) in zip(axes.flat, _pairs):
            ax.plot(t, data, linewidth=1.2)
            ax.set_title(lbl); ax.set_xlabel("t (s)"); ax.grid(True, alpha=0.3)
        plt.tight_layout()

        fig2, axes2 = plt.subplots(2, 3, figsize=(15, 8))
        fig2.suptitle(f"{uav_name} – Attitude & Angular Rates", fontsize=13)
        _pairs2 = [
            ("φ (deg)",    deg(history["phi"])),
            ("θ (deg)",    deg(history["theta"])),
            ("ψ (deg)",    deg(history["psi"])),
            ("p (deg/s)",  deg(history["p"])),
            ("q (deg/s)",  deg(history["q"])),
            ("r (deg/s)",  deg(history["r"])),
        ]
        for ax, (lbl, data) in zip(axes2.flat, _pairs2):
            ax.plot(t, data, linewidth=1.2)
            ax.set_title(lbl); ax.set_xlabel("t (s)"); ax.grid(True, alpha=0.3)
        plt.tight_layout()

        fig3, axes3 = plt.subplots(1, 4, figsize=(16, 4))
        fig3.suptitle(f"{uav_name} – Control Inputs", fontsize=13)
        for ax, lbl, key in zip(axes3,
                                 ["Elevator", "Aileron", "Rudder", "Throttle"],
                                 ["elevator", "aileron", "rudder", "throttle"]):
            ax.plot(t, history[key], linewidth=1.2)
            ax.set_title(lbl); ax.set_xlabel("t (s)"); ax.grid(True, alpha=0.3)
        plt.tight_layout()

        plt.show()
