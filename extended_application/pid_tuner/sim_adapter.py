"""
sim_adapter.py  –  Bridge between ParamStore and FixedWingSimulator.

Provides two integration modes:

Mode A – Inline (same thread):
    Call `adapter.maybe_apply(sim)` inside the simulation loop.
    Zero threading overhead; best for script usage.

Mode B – Background polling:
    Call `adapter.start_polling(sim, interval=0.1)` once.
    A daemon thread polls ParamStore every `interval` seconds and calls
    sim.att_ctrl.reload_gains() / sim.rate_ctrl.reload_gains() automatically.
    Useful when the simulator runs in its own thread (GUI + sim co-exist).

Usage (Mode A)
--------------
from pid_tuner.param_store   import ParamStore
from pid_tuner.sim_adapter   import SimAdapter
from simulation.simulator    import FixedWingSimulator

store   = ParamStore()
sim     = FixedWingSimulator(aircraft_name="TB2")
adapter = SimAdapter(store)

state = sim.init_step()
for _ in range(3000):
    adapter.maybe_apply(sim)       # hot-reload if params changed
    state = sim.step(0.01)

Usage (Mode B – with GUI)
--------------------------
from pid_tuner.gui_tuner  import PIDTunerGUI
from pid_tuner.sim_adapter import SimAdapter
import threading

store   = ParamStore()
sim     = FixedWingSimulator(aircraft_name="TB2")
adapter = SimAdapter(store)
adapter.start_polling(sim, interval=0.05)

def run_sim():
    sim.run(closed_loop=True)

threading.Thread(target=run_sim, daemon=True).start()
PIDTunerGUI(store=store).run()
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from pid_tuner.param_store import ParamStore


class SimAdapter:
    """
    Connects a ParamStore to a live FixedWingSimulator instance,
    hot-reloading PID gains whenever the store changes.

    Parameters
    ----------
    store       : shared ParamStore
    auto_reset  : if True, reset PID integrators after each gain update
                  (prevents integrator wind-up from stale values)
    """

    def __init__(self, store: ParamStore, auto_reset: bool = True):
        self.store      = store
        self.auto_reset = auto_reset
        self._poll_thread: Optional[threading.Thread] = None
        self._polling   = False
        self._apply_count = 0

    # ------------------------------------------------------------------
    # Mode A – inline call
    # ------------------------------------------------------------------

    def maybe_apply(self, sim) -> bool:
        """
        Apply parameter updates to `sim` if the store has changed.
        Returns True if an update was applied.

        Call this at the start of each simulation step.
        """
        if not self.store.has_update():
            return False
        self._do_apply(sim)
        return True

    # ------------------------------------------------------------------
    # Mode B – background polling
    # ------------------------------------------------------------------

    def start_polling(self, sim, interval: float = 0.05) -> None:
        """
        Start a daemon thread that polls the store every `interval` seconds.

        Parameters
        ----------
        sim      : FixedWingSimulator instance
        interval : polling interval in seconds
        """
        if self._polling:
            return
        self._polling = True
        self._poll_thread = threading.Thread(
            target=self._poll_loop,
            args=(sim, interval),
            daemon=True,
        )
        self._poll_thread.start()

    def stop_polling(self) -> None:
        self._polling = False

    def _poll_loop(self, sim, interval: float) -> None:
        while self._polling:
            if self.store.has_update():
                self._do_apply(sim)
            time.sleep(interval)

    # ------------------------------------------------------------------
    # Core apply logic
    # ------------------------------------------------------------------

    def _do_apply(self, sim) -> None:
        """Reload gains from store into the simulator's control layers."""
        try:
            self.store.apply_to_sim(sim)
            self._apply_count += 1
            if self.auto_reset:
                # Reset integrators to avoid jumps from stale integral state
                sim.att_ctrl.reset()
                sim.rate_ctrl.reset()
        except Exception as e:
            print(f"[SimAdapter] apply error: {e}")

    @property
    def apply_count(self) -> int:
        """Total number of gain updates applied so far."""
        return self._apply_count

    # ------------------------------------------------------------------
    # Convenience: run full simulation with live parameter updates
    # ------------------------------------------------------------------

    def run_with_live_tuning(
        self,
        sim,
        closed_loop: bool = True,
        verbose: bool = True,
    ):
        """
        Run the simulator step-by-step while accepting live parameter
        updates from the store.

        Returns the final AircraftSimState.

        This is a blocking call. Run in a background thread if you need
        the GUI to remain responsive simultaneously.
        """
        import sys, os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

        n_steps = int(sim.duration / sim.dt)
        state   = sim.init_step()
        updates = 0

        for i in range(n_steps):
            if self.maybe_apply(sim):
                updates += 1
                if verbose:
                    gains = self.store.get_all()
                    print(f"[SimAdapter] t={i*sim.dt:.2f}s  "
                          f"gain update #{updates}  "
                          f"PTCH_RATE_P={gains.get('PTCH_RATE_P', '?'):.4f}")
            state = sim.step(sim.dt)

        if verbose:
            print(f"[SimAdapter] Simulation done.  "
                  f"{n_steps} steps, {updates} gain updates applied.")
        return state
