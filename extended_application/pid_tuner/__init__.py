"""
pid_tuner package – PID adaptive tuning framework for FixedWingSimulator.

Modules
-------
param_store  : thread-safe parameter registry (ParamStore)
rl_env       : Gymnasium-compatible PID tuning environment (PIDTuningEnv)
rl_agent     : PPO agent + training entry-point (PPOAgent, train)
gui_tuner    : Tkinter GUI (PIDTunerGUI)
sim_adapter  : bridge between ParamStore and FixedWingSimulator (SimAdapter)

Quick start
-----------
# Launch GUI only
python -m pid_tuner

# Launch GUI pre-loaded with existing config
python -m pid_tuner --axis pitch --config config/control_params.yaml

# Train RL agent (no GUI)
python -m pid_tuner --train --axis pitch --steps 50000

# Run simulation with live GUI tuning
python -m pid_tuner --sim --aircraft TB2 --duration 60
"""

from .param_store import ParamStore, PARAM_GROUPS, DEFAULT_PARAMS
from .param_store import PARAM_RANGES as PARAM_RANGES  # noqa: PLC0414
from .rl_env      import PIDTuningEnv
from .rl_env      import make_env as make_env          # noqa: PLC0414
from .sim_adapter import SimAdapter

__all__ = [
    "ParamStore", "PARAM_GROUPS", "PARAM_RANGES", "DEFAULT_PARAMS",
    "PIDTuningEnv", "make_env",
    "SimAdapter",
]
