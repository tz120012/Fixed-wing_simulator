"""
dashboard.py  –  Interactive Matplotlib widget dashboard.

Provides a real-time interactive simulation dashboard with:
  - Flight mode selector (dropdown)
  - PID gain sliders
  - Pause / Resume / Restart buttons
  - Live state numerical readout

Integrates with FixedWingSimulator.step() for incremental updates.
"""

from __future__ import annotations

import numpy as np
from typing import Optional, Dict, Any

try:
    import matplotlib
    import matplotlib.pyplot as plt
    import matplotlib.widgets as mwidgets
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False


class FixedWingDashboard:
    """
    Interactive real-time simulation dashboard.

    Usage
    -----
    >>> from simulation.simulator import FixedWingSimulator
    >>> from visualization.dashboard import FixedWingDashboard
    >>> sim = FixedWingSimulator("TB2")
    >>> dash = FixedWingDashboard(sim)
    >>> dash.run()
    """

    def __init__(self, simulator, max_steps: int = 5000):
        if not _HAS_MPL:
            raise ImportError("matplotlib is required for the dashboard.")

        self.sim       = simulator
        self.max_steps = max_steps
        self._paused   = False
        self._running  = False

        # History buffers (circular / growing)
        self._t_hist:   list = []
        self._alt_hist: list = []
        self._spd_hist: list = []
        self._phi_hist: list = []
        self._theta_hist: list = []

    # ------------------------------------------------------------------

    def run(self) -> None:
        """Build the dashboard figure and start the interactive loop."""
        matplotlib.use("TkAgg")   # ensure interactive backend
        self._build_figure()
        self.sim.init_step()
        self._running = True
        self._ani = None

        import matplotlib.animation as animation

        def _update(frame):
            if self._paused:
                return

            try:
                state = self.sim.step()
            except Exception as e:
                print(f"[Dashboard] Step error: {e}")
                self._running = False
                return

            t = float(len(self._t_hist)) * self.sim.dt
            self._t_hist.append(t)
            self._alt_hist.append(state.altitude)
            self._spd_hist.append(state.airspeed)
            self._phi_hist.append(np.degrees(state.phi))
            self._theta_hist.append(np.degrees(state.theta))

            # Update text readout
            self._txt_state.set_text(
                f"t = {t:.1f}s\n"
                f"alt = {state.altitude:.1f} m\n"
                f"V   = {state.airspeed:.1f} m/s\n"
                f"φ   = {np.degrees(state.phi):.1f}°\n"
                f"θ   = {np.degrees(state.theta):.1f}°\n"
                f"ψ   = {np.degrees(state.psi):.1f}°\n"
                f"Mode: {self.sim.mode_mgr.current_mode.value}"
            )

            # Update line plots
            t_arr = np.array(self._t_hist)
            self._ln_alt.set_data(t_arr, np.array(self._alt_hist))
            self._ln_spd.set_data(t_arr, np.array(self._spd_hist))
            self._ax_alt.relim(); self._ax_alt.autoscale_view()
            self._ax_spd.relim(); self._ax_spd.autoscale_view()
            self._fig.canvas.draw_idle()

        self._ani = animation.FuncAnimation(
            self._fig, _update, interval=int(self.sim.dt * 1000),
            cache_frame_data=False,
        )
        plt.show()

    # ------------------------------------------------------------------

    def _build_figure(self) -> None:
        self._fig = plt.figure(figsize=(14, 8))
        self._fig.suptitle("Fixed-Wing Simulator Dashboard", fontsize=13)

        # --- Axes layout ----------------------------------------------------
        self._ax_alt = self._fig.add_axes([0.05, 0.55, 0.40, 0.35])
        self._ax_alt.set_title("Altitude (m)"); self._ax_alt.set_xlabel("t (s)")
        self._ax_alt.grid(True, alpha=0.3)
        self._ln_alt, = self._ax_alt.plot([], [], "b-", linewidth=1.2)

        self._ax_spd = self._fig.add_axes([0.55, 0.55, 0.40, 0.35])
        self._ax_spd.set_title("Airspeed (m/s)"); self._ax_spd.set_xlabel("t (s)")
        self._ax_spd.grid(True, alpha=0.3)
        self._ln_spd, = self._ax_spd.plot([], [], "r-", linewidth=1.2)

        # --- State readout --------------------------------------------------
        self._txt_state = self._fig.add_axes([0.05, 0.05, 0.25, 0.40]).text(
            0.05, 0.95, "Initialising...",
            transform=self._fig.axes[-1].transAxes,
            verticalalignment="top", fontfamily="monospace", fontsize=10,
        )
        self._fig.axes[-1].set_visible(False)

        # --- Buttons --------------------------------------------------------
        ax_pause   = self._fig.add_axes([0.40, 0.12, 0.12, 0.06])
        ax_restart = self._fig.add_axes([0.55, 0.12, 0.12, 0.06])
        self._btn_pause   = mwidgets.Button(ax_pause,   "Pause")
        self._btn_restart = mwidgets.Button(ax_restart, "Restart")
        self._btn_pause.on_clicked(self._on_pause)
        self._btn_restart.on_clicked(self._on_restart)

        # --- Mode selector --------------------------------------------------
        ax_mode = self._fig.add_axes([0.40, 0.22, 0.25, 0.08])
        modes   = ["MANUAL", "STABILIZE", "FBW_A", "FBW_B", "AUTO", "LOITER", "RTH"]
        self._radio = mwidgets.RadioButtons(ax_mode, modes, active=4)
        self._radio.on_clicked(self._on_mode_change)

    def _on_pause(self, event):
        self._paused = not self._paused
        self._btn_pause.label.set_text("Resume" if self._paused else "Pause")

    def _on_restart(self, event):
        self._t_hist.clear()
        self._alt_hist.clear()
        self._spd_hist.clear()
        self._phi_hist.clear()
        self._theta_hist.clear()
        self.sim.init_step()
        self._paused = False
        self._btn_pause.label.set_text("Pause")

    def _on_mode_change(self, label: str):
        self.sim.mode_mgr.set_mode_str(label)
