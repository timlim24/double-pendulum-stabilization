"""
hybrid_mpc_lqr
==============
Пакет гібридної системи стабілізації подвійного інвертованого маятника (MPC Swing-Up + LQR Catch).
"""

from .config import SystemConfig, DEFAULT_CONFIG
from .dynamics import DoublePendulumDynamics, wrap_to_pi
from .lqr_controller import LQRController
from .mpc_controller import MPCTrajectoryController
from .simulation_manager import SimulationManager, ControlMode
from .main import HybridDoublePendulumApp, run_app

__all__ = [
    "SystemConfig",
    "DEFAULT_CONFIG",
    "DoublePendulumDynamics",
    "wrap_to_pi",
    "LQRController",
    "MPCTrajectoryController",
    "SimulationManager",
    "ControlMode",
    "HybridDoublePendulumApp",
    "run_app"
]

