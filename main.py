"""
FixedWingSimulator - Main Entry Point
======================================
Run a complete fixed-wing UAV simulation from the command line.

Usage examples
--------------
# Default: TB2 in AUTO mode, 30 s, minimum-snap trajectory
python main.py

# Predator in FBW_B mode with wind for 60 s
python main.py --aircraft Predator --mode FBW_B --duration 60 --wind SINE

# TB2 4-DOF linear analysis only
python main.py --aircraft TB2 --analysis 4dof

# List available aircraft
python main.py --list-aircraft
"""

import argparse
import sys
import os

# Make sure the src package is importable when running from project root.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from simulation.simulator import FixedWingSimulator
from models.aircraft_database import AIRCRAFT_NAMES


def parse_args():
    parser = argparse.ArgumentParser(
        description="Fixed-Wing UAV Simulation and Control Platform",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--aircraft",
        default="TB2",
        choices=AIRCRAFT_NAMES,
        help="Aircraft name from the database (default: TB2)",
    )
    parser.add_argument(
        "--mode",
        default="AUTO",
        choices=["MANUAL", "STABILIZE", "FBW_A", "FBW_B", "AUTO", "LOITER", "RTH"],
        help="Initial flight mode (default: AUTO)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Simulation duration in seconds (default: 30)",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.01,
        help="Simulation time step in seconds (default: 0.01)",
    )
    parser.add_argument(
        "--wind",
        default="NONE",
        choices=["NONE", "FIXED", "SINE", "RANDOMSINE"],
        help="Wind model type (default: NONE)",
    )
    parser.add_argument(
        "--traj",
        default="minimum_snap",
        choices=["minimum_snap", "minimum_jerk", "minimum_accel", "minimum_vel", "hover"],
        help="Trajectory type (default: minimum_snap)",
    )
    parser.add_argument(
        "--analysis",
        default=None,
        choices=["4dof", "6dof"],
        help="Run open-loop analysis instead of closed-loop simulation",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable visualization (useful for batch runs)",
    )
    parser.add_argument(
        "--list-aircraft",
        action="store_true",
        help="List all available aircraft and exit",
    )
    parser.add_argument(
        "--config-dir",
        default=os.path.join(os.path.dirname(__file__), "config"),
        help="Path to config directory (default: ./config)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.list_aircraft:
        print("Available aircraft in the database:")
        for name in AIRCRAFT_NAMES:
            print(f"  {name}")
        return

    print(f"[FixedWingSimulator] Aircraft : {args.aircraft}")
    print(f"[FixedWingSimulator] Mode     : {args.mode}")
    print(f"[FixedWingSimulator] Duration : {args.duration} s  |  dt = {args.dt} s")
    print(f"[FixedWingSimulator] Wind     : {args.wind}")
    print(f"[FixedWingSimulator] Trajectory: {args.traj}")
    print()

    sim = FixedWingSimulator(
        aircraft_name=args.aircraft,
        config_dir=args.config_dir,
        dt=args.dt,
        duration=args.duration,
        initial_mode=args.mode,
        wind_type=args.wind,
        traj_type=args.traj,
    )

    if args.analysis == "4dof":
        print("[FixedWingSimulator] Running 4-DOF linear analysis...")
        result = sim.run_linear_analysis()
        print(result.summary())
        if not args.no_plot:
            result.plot()
    elif args.analysis == "6dof":
        print("[FixedWingSimulator] Running 6-DOF open-loop simulation...")
        result = sim.run(closed_loop=False)
        if not args.no_plot:
            result.visualize()
    else:
        print("[FixedWingSimulator] Running closed-loop simulation...")
        result = sim.run(closed_loop=True)
        print(result.summary())
        if not args.no_plot:
            result.visualize()


if __name__ == "__main__":
    main()
