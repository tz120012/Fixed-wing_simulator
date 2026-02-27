"""
pid_tuner/__main__.py  –  Command-line entry point.

Usage
-----
# GUI only (default)
python -m pid_tuner
python -m pid_tuner --axis roll
python -m pid_tuner --config config/control_params.yaml

# Train RL agent (no GUI)
python -m pid_tuner --train --axis pitch --steps 50000 --save checkpoints/pitch.pt

# Run full simulation with live GUI tuning (GUI + sim in parallel)
python -m pid_tuner --sim --aircraft TB2 --duration 60 --mode AUTO
"""

from __future__ import annotations

import argparse
import os
import sys
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))          # .../extended_application/pid_tuner
_EXT  = os.path.dirname(_HERE)                              # .../extended_application
_ROOT = os.path.dirname(_EXT)                               # .../FixedWingSimulator
sys.path.insert(0, os.path.join(_ROOT, "src"))              # FixedWingSimulator/src
sys.path.insert(0, _EXT)                                    # extended_application (for pid_tuner pkg)


def _parse():
    p = argparse.ArgumentParser(
        prog="python -m pid_tuner",
        description="PID Adaptive Tuning Framework for FixedWingSimulator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--axis",     default="pitch",
                   choices=["pitch", "roll", "yaw"],
                   help="Axis to tune (default: pitch)")
    p.add_argument("--config",   default=None,
                   help="Pre-load YAML parameter file")
    p.add_argument("--train",    action="store_true",
                   help="Run RL training instead of GUI")
    p.add_argument("--steps",    type=int, default=50_000,
                   help="Total RL training steps (default: 50000)")
    p.add_argument("--save",     default=None,
                   help="Checkpoint save path for RL training")
    p.add_argument("--sim",      action="store_true",
                   help="Launch full 6-DOF simulation alongside GUI")
    p.add_argument("--aircraft", default="TB2",
                   help="Aircraft name for --sim mode (default: TB2)")
    p.add_argument("--duration", type=float, default=60.0,
                   help="Simulation duration in seconds (default: 60)")
    p.add_argument("--mode",     default="STABILIZE",
                   help="Flight mode for --sim (default: STABILIZE)")
    return p.parse_args()


def main():
    args = _parse()

    # -----------------------------------------------------------------
    # Mode 1: RL training (headless)
    # -----------------------------------------------------------------
    if args.train:
        try:
            from pid_tuner.rl_agent import train
        except ImportError as e:
            print(f"[pid_tuner] PyTorch required for RL training: {e}")
            sys.exit(1)

        save_path = args.save or os.path.join(
            _HERE, "checkpoints", f"{args.axis}_ppo.pt")
        print(f"[pid_tuner] Training PPO for axis='{args.axis}', "
              f"steps={args.steps}, save='{save_path}'")
        agent = train(axis=args.axis, total_steps=args.steps,
                      save_path=save_path, verbose=True)
        print(f"[pid_tuner] Training complete. Checkpoint: {save_path}")
        return

    # -----------------------------------------------------------------
    # Mode 2: GUI (with optional live simulation)
    # -----------------------------------------------------------------
    from pid_tuner.param_store import ParamStore
    from pid_tuner.gui_tuner   import PIDTunerGUI

    store = ParamStore()
    if args.config and os.path.isfile(args.config):
        store.load_yaml(args.config)
        print(f"[pid_tuner] Loaded config: {args.config}")
    elif not args.config:
        # Try default config location
        default_cfg = os.path.join(_ROOT, "config", "control_params.yaml")
        if os.path.isfile(default_cfg):
            store.load_yaml(default_cfg)

    # Launch 6-DOF simulation in background thread if requested
    if args.sim:
        from pid_tuner.sim_adapter import SimAdapter
        try:
            from simulation.simulator import FixedWingSimulator
        except ImportError as e:
            print(f"[pid_tuner] Cannot import simulator: {e}")
            sys.exit(1)

        sim     = FixedWingSimulator(
            aircraft_name=args.aircraft,
            dt=0.02,
            duration=args.duration,
            initial_mode=args.mode,
        )
        adapter = SimAdapter(store, auto_reset=True)

        def _sim_thread():
            print(f"[pid_tuner] Simulation started: {args.aircraft} / "
                  f"{args.mode} / {args.duration}s")
            final = adapter.run_with_live_tuning(sim, closed_loop=True)
            print(f"[pid_tuner] Simulation ended. "
                  f"alt={final.altitude:.1f}m  "
                  f"spd={final.airspeed:.1f}m/s")

        threading.Thread(target=_sim_thread, daemon=True).start()

    # Start GUI (blocking)
    app = PIDTunerGUI(store=store, axis=args.axis)
    app.run()


if __name__ == "__main__":
    main()
