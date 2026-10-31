"""
down_lqr_controller.py
======================
Контролер активного заспокоєння та центрування візка в нижній точці (Down-LQR).
Гасить коливання маятника біля theta = pi та утримує візок у точці x = 0.
"""

from typing import Tuple

import numpy as np
import scipy.linalg

try:
    from .config import DEFAULT_CONFIG, SinglePendulumConfig
    from .dynamics import wrap_to_pi
except ImportError:
    from config import DEFAULT_CONFIG, SinglePendulumConfig
    from dynamics import wrap_to_pi


class DownLQRController:
    """
    Контролер активного заспокоєння одинарного маятника в нижньому положенні.
    """

    def __init__(self, cfg: SinglePendulumConfig = DEFAULT_CONFIG):
        self.cfg = cfg
        self.K_down = self.compute_down_gains()

    def compute_down_gains(self) -> np.ndarray:
        """
        Розрахунок коефіцієнтів LQR для лінеаризації навколо нижньої точки (theta = pi).
        """
        M = self.cfg.M
        m = self.cfg.m
        l = self.cfg.l
        I = self.cfg.I
        g = self.cfg.g

        # Для нижньої точки (theta = pi, sin(pi+alpha) ~ -alpha, cos(pi+alpha) ~ -1)
        D = I * (M + m) - (m * l) ** 2
        a32 = (m * l) ** 2 * g / D
        a42 = -(M + m) * m * g * l / D
        b3 = I / D
        b4 = (m * l) / D

        A = np.array(
            [
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [0.0, a32, 0.0, 0.0],
                [0.0, a42, 0.0, 0.0],
            ],
            dtype=float,
        )

        B = np.array([[0.0], [0.0], [b3], [b4]], dtype=float)
        Q = np.diag(
            [
                self.cfg.down_q_x,
                self.cfg.down_q_theta,
                self.cfg.down_q_v,
                self.cfg.down_q_omega,
            ]
        )
        R = np.array([[self.cfg.down_r_effort]])

        try:
            P = scipy.linalg.solve_continuous_are(A, B, Q, R)
            K = (np.linalg.inv(R) @ B.T @ P).flatten()
        except Exception:
            # Надійні перевірені коефіцієнти прямого демпфування
            K = np.array([12.0, -18.0, 6.0, -4.5])
        return K

    def compute_force(
        self, state: np.ndarray, target_x: float = 0.0
    ) -> Tuple[float, float, bool]:
        """
        Обчислення сили занулення візка та гасіння коливань маятника.
        Повертає: (u_sat, u_raw, is_sat)
        """
        x, th, v, w = state
        # Відхилення від нижньої точки (де theta = pi)
        # angle_from_bottom: 0 коли висить строго вниз
        th_w = wrap_to_pi(th)
        alpha = th_w - np.pi if th_w >= 0 else th_w + np.pi
        x_err = x - target_x

        u_raw = float(
            -(
                self.K_down[0] * x_err
                + self.K_down[1] * alpha
                + self.K_down[2] * v
                + self.K_down[3] * w
            )
        )
        u_sat = float(np.clip(u_raw, -self.cfg.u_max * 0.6, self.cfg.u_max * 0.6))
        is_sat = bool(abs(u_sat - u_raw) > 1e-3)
        return u_sat, u_raw, is_sat

    def is_zeroed(self, state: np.ndarray, target_x: float = 0.0) -> bool:
        """Перевіряє, чи заспокоївся маятник у нульовому стані."""
        x, th, v, w = state
        th_w = wrap_to_pi(th)
        alpha = th_w - np.pi if th_w >= 0 else th_w + np.pi
        return bool(
            abs(x - target_x) < 0.025
            and abs(v) < 0.03
            and abs(alpha) < np.radians(3.0)
            and abs(w) < 0.20
        )
