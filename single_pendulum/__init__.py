"""
single_pendulum package
=======================
Симуляція та керування одинарним перевернутим маятником на візку (LQR + MPC Swing-Up).
"""

from .config import SinglePendulumConfig, DEFAULT_CONFIG
from .lqr_controller import SinglePendulumLQR
from .mpc_controller import SinglePendulumMPCController
from .mpc_generator import generate_single_pendulum_trajectory
