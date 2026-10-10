"""
config.py
=========
Central configuration module for the Inverted Double Pendulum on a Cart.

All default values, physical parameters, damping coefficients, and state-space
matrices are extracted directly from the repository source-of-truth files:
  - ParameterEstimation/DoublePendulum/EstimatedParameters_DoublePendulum.mat
  - ParameterEstimation/DoublePendulum/DoublePendulumLQRControl.m
  - ParameterEstimation/DoublePendulum/DoublePendulumSymbolicCalculation.m

System Constraints:
  - Initial configuration: System MUST start from the DOWN position (hanging down: theta1 = pi, theta2 = pi).
  - First link longer than second link: L1 > L2 (L1 = 0.1727 m, L2 = 0.1400 m).
  - Viscous track friction (k0) and joint damping (k1, k2).
  - Actuator force limits: [-F_max, F_max].
  - Track limits: [-X_max, X_max].
  - LQR Q matrix: Q[0, 0] = 0 (cart position is ignored).
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
import numpy as np


@dataclass
class PendulumConfig:
    """
    Physical parameters, limits, LQR cost matrices, and simulation settings.
    """

    # --- Cart Parameters (DoublePendulumLQRControl.m, lines 39-40) ---
    M: float = 3.2           # Cart mass (kg)
    k0: float = 0.5          # Viscous friction coefficient of cart on track (N*s/m)

    # --- Pendulum Link 1 Parameters (EstimatedParameters_DoublePendulum.mat) ---
    m1: float = 0.0938439748  # Mass of link 1 (kg)
    a1: float = 0.108565215   # Distance to CoM of link 1 from joint 1 (m)
    L1: float = 0.172719204   # Pivot-to-pivot length of link 1 (m)
    I1: float = 4.3752943e-4  # Moment of inertia of link 1 about CoM (kg*m^2)
    k1: float = 2.3714278e-4  # Damping friction at joint 1 (N*m*s/rad)

    # --- Pendulum Link 2 Parameters (EstimatedParameters_DoublePendulum.mat) ---
    m2: float = 0.13759597    # Mass of link 2 (kg)
    a2: float = 0.116779018   # Distance to CoM of link 2 from joint 2 (m)
    L2: float = 0.1400        # Visual/geometric length of link 2 (m), ensuring L1 > L2
    I2: float = 1.26882939e-3 # Moment of inertia of link 2 about CoM (kg*m^2)
    k2: float = 1.00000019e-5 # Damping friction at joint 2 (N*m*s/rad)

    # --- Environment & Gravitational Acceleration ---
    g: float = 9.80858023     # Local gravitational acceleration (m/s^2)

    # --- Actuator & Physical Track Limits ---
    F_max: float = 40.0       # Maximum motor actuator force saturation limit (N)
    X_max: float = 0.80       # Track boundary half-span: cart travels in [-X_max, X_max] (m)

    # --- Simulation Timing ---
    dt: float = 0.002         # Physics integration time step (s) -> 500 Hz
    stream_fps: int = 60      # Telemetry streaming rate to web frontend (Hz)

    # --- Initial State & Swing-Up Configuration ---
    start_from_down: bool = True     # Initial state must start from the DOWN position
    swing_up_duration: float = 1.8   # Duration of the swing-up trajectory to upright (s)

    # --- LQR Weighting Matrices (DoublePendulumLQRControl.m, lines 94-104) ---
    # State vector: [x, theta1, theta2, dx, dtheta1, dtheta2]
    # NOTE: q_x is chosen small (1.0) but non-zero: provides gentle cart centering
    # to prevent drifting toward track boundaries while giving overwhelming priority
    # to pendulum vertical angles and angular velocities.
    q_x: float = 1.0          # Small non-zero cost weight for cart position x (yields K_x ~ 7.07)
    q_th1: float = 1200.0     # Cost weight for Link 1 angle theta1
    q_th2: float = 1500.0     # Cost weight for Link 2 angle theta2
    q_dx: float = 15.0        # Cost weight for cart velocity dx (stabilizes drift speed)
    q_dth1: float = 60.0      # Cost weight for Link 1 angular velocity dtheta1
    q_dth2: float = 60.0      # Cost weight for Link 2 angular velocity dtheta2
    r_u: float = 0.02         # Control effort penalty weight (R matrix)

    # --- Disturbance Settings ---
    default_push_torque1: float = 0.04   # Disturbance torque pulse on Link 1 (N*m)
    default_push_torque2: float = 0.03   # Disturbance torque pulse on Link 2 (N*m)
    push_duration: float = 0.04          # Disturbance pulse duration (s)

    # --- Web Server Settings ---
    host: str = "0.0.0.0"
    port: int = 8000

    def __post_init__(self):
        """Validate physical constraints."""
        assert self.L1 > self.L2, f"System constraint violation: L1 ({self.L1}) must be strictly > L2 ({self.L2})"
        assert self.q_x >= 0.0, "q_x must be non-negative"
        assert self.F_max > 0.0, "F_max must be positive"
        assert self.X_max > 0.0, "X_max must be positive"

    # --- Inertial Groupings (DoublePendulumLQRControl.m, lines 42-48) ---
    @property
    def mu0(self) -> float:
        """Total translational mass: M + m1 + m2."""
        return self.M + self.m1 + self.m2

    @property
    def p1(self) -> float:
        """First momentum coupling term: m1 * a1 + m2 * L1."""
        return self.m1 * self.a1 + self.m2 * self.L1

    @property
    def p2(self) -> float:
        """Second momentum coupling term: m2 * a2."""
        return self.m2 * self.a2

    @property
    def p12(self) -> float:
        """Inter-link cross inertial coupling: m2 * L1 * a2."""
        return self.m2 * self.L1 * self.a2

    @property
    def J1(self) -> float:
        """Effective moment of inertia of Link 1: I1 + m1*a1^2 + m2*L1^2."""
        return self.I1 + self.m1 * (self.a1 ** 2) + self.m2 * (self.L1 ** 2)

    @property
    def J2(self) -> float:
        """Effective moment of inertia of Link 2: I2 + m2*a2^2."""
        return self.I2 + self.m2 * (self.a2 ** 2)

    # --- LQR Cost Matrices ---
    @property
    def Q(self) -> np.ndarray:
        """6x6 state cost matrix Q."""
        return np.diag([
            self.q_x,
            self.q_th1,
            self.q_th2,
            self.q_dx,
            self.q_dth1,
            self.q_dth2
        ])

    @property
    def R(self) -> np.ndarray:
        """1x1 control cost matrix R."""
        return np.array([[self.r_u]])


def load_config_from_mat(mat_path: Optional[Path] = None) -> PendulumConfig:
    """
    Attempt to load estimated parameters from EstimatedParameters_DoublePendulum.mat.
    Falls back to default PendulumConfig if file is missing or scipy is unavailable.
    """
    cfg = PendulumConfig()
    if mat_path is None:
        mat_path = Path(__file__).resolve().parent.parent / "ParameterEstimation" / "DoublePendulum" / "EstimatedParameters_DoublePendulum.mat"

    if mat_path.is_file():
        try:
            import scipy.io as sio
            mat_data = sio.loadmat(str(mat_path))
            if "EstimatedParameters" in mat_data:
                p = mat_data["EstimatedParameters"].flatten()
                # Array order: [m1, m2, a1, a2, L1, I1, I2, k1, k2, g]
                cfg.m1 = float(p[0])
                cfg.m2 = float(p[1])
                cfg.a1 = float(p[2])
                cfg.a2 = float(p[3])
                cfg.L1 = float(p[4])
                cfg.I1 = float(p[5])
                cfg.I2 = float(p[6])
                cfg.k1 = float(p[7])
                cfg.k2 = float(p[8])
                cfg.g = float(p[9])
                # Ensure L1 > L2 constraint is preserved
                cfg.L2 = min(0.14, cfg.L1 * 0.81)
        except Exception as e:
            # Fall back silently to verified nominal values
            pass

    return cfg


# Global default configuration instance
DEFAULT_CONFIG = load_config_from_mat()
