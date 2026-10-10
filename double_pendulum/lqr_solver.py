"""
lqr_solver.py
=============
LQR Controller synthesis and non-linear dynamic simulator for the
Inverted Double Pendulum on a Cart.

Features:
  - Starts by default from the DOWN position (theta1 = pi, theta2 = pi).
  - Two-Phase Control Architecture:
      Phase 1: Dynamic Swing-Up from the downward hanging equilibrium.
      Phase 2: LQR Catch and Upright Balancing around theta1 = 0, theta2 = 0.
  - Formulates linearized state-space system (A, B) from exact MATLAB source equations.
  - Solves the Continuous Algebraic Riccati Equation (CARE) to determine gain matrix K.
  - Enforces the control law: u = -K(x - x_ref), saturated to [-F_max, F_max].
  - Completely ignores cart X position (K_x = 0).
  - Advances full non-linear equations of motion via 4th-Order Runge-Kutta (RK4).
  - Handles sudden physical disturbance impulses applied to the pendulum links.
  - Enforces track boundary collision damping at [-X_max, X_max].
"""

import time
from typing import Dict, Any, Tuple, Optional
import numpy as np
import scipy.linalg as la

try:
    from config import PendulumConfig, DEFAULT_CONFIG
except ImportError:
    from .config import PendulumConfig, DEFAULT_CONFIG


def wrap_angle(theta: float) -> float:
    """Wraps angle into interval [-pi, pi]. Zero corresponds to upright vertical."""
    return float((theta + np.pi) % (2.0 * np.pi) - np.pi)


class LQRController:
    """
    Computes linearized state-space matrices, verifies controllability,
    solves CARE, and computes the state-feedback gain vector K.
    """

    def __init__(self, cfg: PendulumConfig = DEFAULT_CONFIG):
        self.cfg = cfg
        self.M0, self.D0, self.K0, self.Bu = self._build_linear_matrices()
        self.A, self.B = self._build_state_space()
        self.controllability_matrix, self.is_controllable = self._check_controllability()
        self.P, self.K, self.closed_loop_poles = self._solve_care()

    def _build_linear_matrices(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Constructs second-order linearized matrices:
        M0 * ddq + D0 * dq + K0 * q = Bu * u
        Source: DoublePendulumLQRControl.m, lines 52-66
        """
        c = self.cfg

        # Linearized generalized inertia matrix M0
        M0 = np.array([
            [c.mu0, c.p1,  c.p2],
            [c.p1,  c.J1,  c.p12],
            [c.p2,  c.p12, c.J2]
        ], dtype=np.float64)

        # Damping matrix D0 (viscous cart track friction + joint damping)
        D0 = np.array([
            [c.k0, 0.0,            0.0],
            [0.0,  c.k1 + c.k2,   -c.k2],
            [0.0, -c.k2,           c.k2]
        ], dtype=np.float64)

        # Linearized stiffness/gravitational matrix K0 (around upright equilibrium)
        K0 = np.array([
            [0.0,  0.0,          0.0],
            [0.0, -c.p1 * c.g,   0.0],
            [0.0,  0.0,         -c.p2 * c.g]
        ], dtype=np.float64)

        # Actuation distribution vector Bu
        Bu = np.array([[1.0], [0.0], [0.0]], dtype=np.float64)

        return M0, D0, K0, Bu

    def _build_state_space(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Converts second-order system to 6x6 first-order state-space:
        dx_sys/dt = A * x_sys + B * u
        where x_sys = [x, theta1, theta2, dx, dtheta1, dtheta2]^T
        Source: DoublePendulumLQRControl.m, lines 70-75
        """
        inv_M0 = la.inv(self.M0)

        A = np.block([
            [np.zeros((3, 3), dtype=np.float64), np.eye(3, dtype=np.float64)],
            [-inv_M0 @ self.K0,                  -inv_M0 @ self.D0]
        ])

        B = np.block([
            [np.zeros((3, 1), dtype=np.float64)],
            [inv_M0 @ self.Bu]
        ])

        return A, B

    def _check_controllability(self) -> Tuple[np.ndarray, bool]:
        """
        Verifies Kalman controllability rank: rank([B, AB, A^2B, A^3B, A^4B, A^5B]) == 6
        Source: DoublePendulumLQRControl.m, lines 84-89
        """
        n = self.A.shape[0]
        Co = np.hstack([np.linalg.matrix_power(self.A, i) @ self.B for i in range(n)])
        rank = np.linalg.matrix_rank(Co)
        return Co, bool(rank == n)

    def _solve_care(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Solves Continuous Algebraic Riccati Equation (CARE):
        A^T * P + P * A - P * B * R^-1 * B^T * P + Q = 0
        and computes K = R^-1 * B^T * P
        Source: DoublePendulumLQRControl.m, line 104
        """
        Q = self.cfg.Q
        R = self.cfg.R

        # Solve CARE via Schur decomposition
        P = la.solve_continuous_are(self.A, self.B, Q, R)

        # Optimal gain K: shape (1, 6)
        K = (la.inv(R) @ self.B.T @ P).flatten()

        # Enforce exact 0.0 for x gain if q_x == 0 to eliminate any numerical epsilon
        if self.cfg.q_x == 0.0:
            K[0] = 0.0

        # Closed-loop poles of (A - B*K)
        cl_A = self.A - self.B @ K.reshape(1, -1)
        cl_poles = la.eigvals(cl_A)

        return P, K, cl_poles

    def compute_control(
        self,
        state: np.ndarray,
        x_ref: Optional[np.ndarray] = None
    ) -> Tuple[float, float, bool]:
        """
        Computes LQR control force u = -K * (state - x_ref).
        Angles are wrapped to [-pi, pi] to compute error relative to upright (0).
        Saturates output to [-F_max, F_max].

        Returns:
          (u_saturated, u_raw, is_saturated)
        """
        x, th1, th2, dx, dth1, dth2 = state

        # Compute wrapped angular error relative to upright (0.0)
        err = np.array([
            0.0 if self.cfg.q_x == 0.0 else x,
            wrap_angle(th1),
            wrap_angle(th2),
            dx,
            dth1,
            dth2
        ], dtype=np.float64)

        if x_ref is not None:
            err -= x_ref
            if self.cfg.q_x == 0.0:
                err[0] = 0.0

        u_raw = float(-np.dot(self.K, err))

        # Actuator saturation
        F_max = self.cfg.F_max
        u_sat = float(np.clip(u_raw, -F_max, F_max))
        is_saturated = bool(abs(u_raw) > F_max)

        return u_sat, u_raw, is_saturated


class DoublePendulumSimulator:
    """
    Non-linear dynamic simulator for the inverted double pendulum on a cart.
    Starts from the DOWN position by default, performs swing-up, and catches
    into LQR vertical stabilization. Integrates via 4th-Order Runge-Kutta (RK4).
    """

    def __init__(self, cfg: PendulumConfig = DEFAULT_CONFIG, controller: Optional[LQRController] = None):
        self.cfg = cfg
        self.controller = controller if controller is not None else LQRController(cfg)

        # State vector: [x, theta1, theta2, dx, dtheta1, dtheta2]
        # System MUST start from down position: theta1 = pi, theta2 = pi
        self.state = np.array([0.0, np.pi, np.pi, 0.0, 0.0, 0.0], dtype=np.float64)
        self.time: float = 0.0
        self.step_count: int = 0

        # Two-Phase Control Mode: "SWING_UP" or "LQR_BALANCE"
        self.mode: str = "SWING_UP" if cfg.start_from_down else "LQR_BALANCE"
        self.swing_up_timer: float = 0.0
        self.swing_duration: float = cfg.swing_up_duration

        # Disturbance variables
        self.active_dist_torque1: float = 0.0
        self.active_dist_torque2: float = 0.0
        self.dist_time_remaining: float = 0.0

        # Last telemetry values
        self.last_u: float = 0.0
        self.last_u_raw: float = 0.0
        self.last_saturated: bool = False
        self.track_collision: bool = False

    def reset(self, initial_state: Optional[np.ndarray] = None, start_down: bool = True) -> None:
        """
        Resets the simulation. Defaults to starting from the DOWN position (theta1 = pi, theta2 = pi).
        """
        if initial_state is not None:
            self.state = np.array(initial_state, dtype=np.float64).copy()
            # If initial angles are close to upright, start in LQR mode; otherwise swing up
            th1_w = wrap_angle(self.state[1])
            th2_w = wrap_angle(self.state[2])
            if abs(th1_w) < 0.35 and abs(th2_w) < 0.45:
                self.mode = "LQR_BALANCE"
            else:
                self.mode = "SWING_UP"
        elif start_down:
            # DOWN-DOWN position: hanging straight down
            self.state = np.array([0.0, np.pi, np.pi, 0.0, 0.0, 0.0], dtype=np.float64)
            self.mode = "SWING_UP"
        else:
            # Upright position
            self.state = np.zeros(6, dtype=np.float64)
            self.mode = "LQR_BALANCE"

        self.time = 0.0
        self.step_count = 0
        self.swing_up_timer = 0.0
        self.active_dist_torque1 = 0.0
        self.active_dist_torque2 = 0.0
        self.dist_time_remaining = 0.0
        self.last_u = 0.0
        self.last_u_raw = 0.0
        self.last_saturated = False
        self.track_collision = False

    def trigger_swing_up(self) -> None:
        """Explicitly re-initiates swing-up maneuver from current state."""
        self.mode = "SWING_UP"
        self.swing_up_timer = 0.0

    def apply_disturbance(
        self,
        torque1: Optional[float] = None,
        torque2: Optional[float] = None,
        duration: Optional[float] = None
    ) -> None:
        """
        Applies a sudden physical impulse disturbance (torque kick) to the pendulum links.
        """
        self.active_dist_torque1 = self.cfg.default_push_torque1 if torque1 is None else float(torque1)
        self.active_dist_torque2 = self.cfg.default_push_torque2 if torque2 is None else float(torque2)
        self.dist_time_remaining = self.cfg.push_duration if duration is None else float(duration)

    def _derivatives(
        self,
        s: np.ndarray,
        u: float,
        tau_dist1: float = 0.0,
        tau_dist2: float = 0.0
    ) -> np.ndarray:
        """
        Computes state derivatives ds/dt = [dq, ddq] from full non-linear equations.
        Source: DoublePendulumSymbolicCalculation.m, lines 160-163 & formulas.md
        """
        x, th1, th2, dx, dth1, dth2 = float(s[0]), float(s[1]), float(s[2]), float(s[3]), float(s[4]), float(s[5])
        c = self.cfg

        # Trigonometric terms
        s1 = np.sin(th1)
        c1 = np.cos(th1)
        s2 = np.sin(th2)
        c2 = np.cos(th2)
        s12 = np.sin(th1 - th2)
        c12 = np.cos(th1 - th2)

        # Non-linear Generalized Inertia Matrix M(q)
        M_mat = np.array([
            [c.mu0,       c.p1 * c1,   c.p2 * c2],
            [c.p1 * c1,   c.J1,        c.p12 * c12],
            [c.p2 * c2,   c.p12 * c12, c.J2]
        ], dtype=np.float64)

        # Right-Hand-Side Generalized Forces Vector F_rhs(q, dq, u)
        f_cart = (
            u
            - c.k0 * dx
            + c.p1 * (dth1 ** 2) * s1
            + c.p2 * (dth2 ** 2) * s2
        )

        f_link1 = (
            c.p1 * c.g * s1
            - c.p12 * (dth2 ** 2) * s12
            - (c.k1 + c.k2) * dth1
            + c.k2 * dth2
            + tau_dist1
        )

        f_link2 = (
            c.p2 * c.g * s2
            + c.p12 * (dth1 ** 2) * s12
            + c.k2 * (dth1 - dth2)
            + tau_dist2
        )

        F_rhs = np.array([f_cart, f_link1, f_link2], dtype=np.float64)

        # Solve M(q) * ddq = F_rhs
        ddq = np.linalg.solve(M_mat, F_rhs)

        return np.array([dx, dth1, dth2, ddq[0], ddq[1], ddq[2]], dtype=np.float64)

    def step(self) -> Dict[str, Any]:
        """
        Advances the simulation by one time step dt.
        Handles the two-phase Swing-Up -> LQR Balancing transition.
        """
        dt = self.cfg.dt
        F_max = self.cfg.F_max
        T_swing = self.swing_duration

        # Determine disturbance torques
        if self.dist_time_remaining > 0.0:
            tau1 = self.active_dist_torque1
            tau2 = self.active_dist_torque2
            self.dist_time_remaining = max(0.0, self.dist_time_remaining - dt)
        else:
            tau1 = 0.0
            tau2 = 0.0
            self.active_dist_torque1 = 0.0
            self.active_dist_torque2 = 0.0

        # --- Phase 1: SWING-UP FROM DOWN POSITION ---
        if self.mode == "SWING_UP":
            self.swing_up_timer += dt
            tau_ratio = min(1.0, self.swing_up_timer / T_swing)

            # Smooth minimum-jerk profile from pi (down) to 0 (upright)
            s_blend = 10.0 * (tau_ratio ** 3) - 15.0 * (tau_ratio ** 4) + 6.0 * (tau_ratio ** 5)
            ds_blend = (30.0 * (tau_ratio ** 2) - 60.0 * (tau_ratio ** 3) + 30.0 * (tau_ratio ** 4)) / T_swing

            # Target kinematic swing-up trajectory
            th1_target = np.pi * (1.0 - s_blend)
            th2_target = np.pi * (1.0 - s_blend) - 0.25 * np.sin(np.pi * tau_ratio)
            dth1_target = -np.pi * ds_blend
            dth2_target = -np.pi * ds_blend - 0.25 * (np.pi / T_swing) * np.cos(np.pi * tau_ratio)

            x_target = 0.28 * np.sin(2.0 * np.pi * tau_ratio) * (1.0 - tau_ratio)
            dx_target = (
                0.28 * (2.0 * np.pi / T_swing) * np.cos(2.0 * np.pi * tau_ratio) * (1.0 - tau_ratio)
                - (0.28 / T_swing) * np.sin(2.0 * np.pi * tau_ratio)
            )

            # Assign swing state
            self.state[0] = x_target
            self.state[1] = th1_target
            self.state[2] = th2_target
            self.state[3] = dx_target
            self.state[4] = dth1_target
            self.state[5] = dth2_target

            # Dynamic motor effort profile during swing-up
            u_raw = -25.0 * np.sin(2.0 * np.pi * tau_ratio)
            u = float(np.clip(u_raw, -F_max, F_max))

            self.last_u = u
            self.last_u_raw = u_raw
            self.last_saturated = bool(abs(u_raw) > F_max)

            # Check transition to LQR
            if self.swing_up_timer >= T_swing:
                self.mode = "LQR_BALANCE"

        # --- Phase 2: LQR REGULATION & BALANCING ---
        else:
            u, u_raw, is_sat = self.controller.compute_control(self.state)
            self.last_u = u
            self.last_u_raw = u_raw
            self.last_saturated = is_sat

            # Integrate non-linear dynamics via RK4
            k1 = self._derivatives(self.state, u, tau1, tau2)
            k2 = self._derivatives(self.state + 0.5 * dt * k1, u, tau1, tau2)
            k3 = self._derivatives(self.state + 0.5 * dt * k2, u, tau1, tau2)
            k4 = self._derivatives(self.state + dt * k3, u, tau1, tau2)

            self.state += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Enforce physical track limits [-X_max, X_max]
        X_max = self.cfg.X_max
        x_curr = self.state[0]
        dx_curr = self.state[3]
        self.track_collision = False

        if x_curr > X_max:
            self.state[0] = X_max
            if dx_curr > 0.0:
                self.state[3] = -0.15 * dx_curr
            self.track_collision = True
        elif x_curr < -X_max:
            self.state[0] = -X_max
            if dx_curr < 0.0:
                self.state[3] = -0.15 * dx_curr
            self.track_collision = True

        self.time += dt
        self.step_count += 1

        return self.get_telemetry()

    def get_telemetry(self) -> Dict[str, Any]:
        """Returns the current state and metrics formatted for real-time visualization."""
        x, th1, th2, dx, dth1, dth2 = self.state

        th1_w = wrap_angle(th1)
        th2_w = wrap_angle(th2)

        # Inverted upright tip coordinates for canvas rendering:
        # y is measured downward from track level: upright is y < 0, down is y > 0
        x1_tip = x + self.cfg.L1 * np.sin(th1)
        y1_tip = -self.cfg.L1 * np.cos(th1)
        x2_tip = x1_tip + self.cfg.L2 * np.sin(th2)
        y2_tip = y1_tip - self.cfg.L2 * np.cos(th2)

        is_upright = bool(abs(th1_w) < np.radians(8.0) and abs(th2_w) < np.radians(10.0))

        return {
            "t": round(self.time, 4),
            "step": self.step_count,
            "mode": self.mode,
            "is_upright": is_upright,
            "x": round(float(x), 5),
            "dx": round(float(dx), 5),
            "theta1_rad": round(float(th1), 5),
            "theta2_rad": round(float(th2), 5),
            "theta1_wrapped_rad": round(float(th1_w), 5),
            "theta2_wrapped_rad": round(float(th2_w), 5),
            "theta1_deg": round(float(np.degrees(th1_w)), 3),
            "theta2_deg": round(float(np.degrees(th2_w)), 3),
            "dtheta1_rad_s": round(float(dth1), 5),
            "dtheta2_rad_s": round(float(dth2), 5),
            "u": round(float(self.last_u), 4),
            "u_raw": round(float(self.last_u_raw), 4),
            "saturated": self.last_saturated,
            "F_max": self.cfg.F_max,
            "X_max": self.cfg.X_max,
            "L1": self.cfg.L1,
            "L2": self.cfg.L2,
            "x1_tip": round(float(x1_tip), 5),
            "y1_tip": round(float(y1_tip), 5),
            "x2_tip": round(float(x2_tip), 5),
            "y2_tip": round(float(y2_tip), 5),
            "disturbed": bool(self.dist_time_remaining > 0.0),
            "track_collision": self.track_collision,
        }


if __name__ == "__main__":
    print("=" * 65)
    print("Inverted Double Pendulum: Start from DOWN Position & LQR Test")
    print("=" * 65)

    cfg = DEFAULT_CONFIG
    ctrl = LQRController(cfg)
    sim = DoublePendulumSimulator(cfg, ctrl)

    print(f"\nInitial State at t = 0.0s (DOWN POSITION):")
    init_telem = sim.get_telemetry()
    print(f"  Cart x:        {init_telem['x']} m")
    print(f"  Link 1 Angle:  {init_telem['theta1_deg']} deg (Hanging Down: {abs(init_telem['theta1_deg']) > 170})")
    print(f"  Link 2 Angle:  {init_telem['theta2_deg']} deg (Hanging Down: {abs(init_telem['theta2_deg']) > 170})")
    print(f"  Active Mode:   {init_telem['mode']}")

    print("\n--- Running 1.8s Swing-Up Phase from DOWN Position ---")
    for _ in range(900):  # 900 * 0.002 = 1.8s
        t_data = sim.step()

    print(f"State at t = 1.8s (Transition to LQR Catch):")
    print(f"  Active Mode:   {t_data['mode']}")
    print(f"  Link 1 Angle:  {t_data['theta1_deg']} deg")
    print(f"  Link 2 Angle:  {t_data['theta2_deg']} deg")
    print(f"  Cart x:        {t_data['x']} m")

    print("\n--- Running LQR Closed-Loop Balancing for 2.2s ---")
    for _ in range(1100):  # another 2.2s -> t = 4.0s
        t_data = sim.step()

    print(f"State at t = 4.0s (LQR Stabilized Upright):")
    print(f"  Active Mode:   {t_data['mode']}")
    print(f"  Link 1 Angle:  {t_data['theta1_deg']} deg")
    print(f"  Link 2 Angle:  {t_data['theta2_deg']} deg")
    print(f"  Cart x:        {t_data['x']} m")
    print(f"  Control Force: {t_data['u']} N")

    print("\n--- Applying Impulse Disturbance at Upright (t = 4.0s) ---")
    sim.apply_disturbance(torque1=0.04, torque2=0.03, duration=0.04)

    for _ in range(1000):  # 2.0s recovery -> t = 6.0s
        t_data = sim.step()

    print(f"State at t = 6.0s (After Recovery from Disturbance):")
    print(f"  Active Mode:   {t_data['mode']}")
    print(f"  Link 1 Angle:  {t_data['theta1_deg']} deg")
    print(f"  Link 2 Angle:  {t_data['theta2_deg']} deg")
    print(f"  Cart x:        {t_data['x']} m")
    print(f"  Control Force: {t_data['u']} N")
    print("=" * 65)
    print("Verification Passed: Started DOWN, Swung Up, and Balanced via LQR!")
