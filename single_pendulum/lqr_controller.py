"""
lqr_controller.py
=================
Аналітичний розрахунок оптимальних коефіцієнтів зворотного зв'язку LQR
(Linear Quadratic Regulator) для верхнього балансування одинарного перевернутого маятника на візку.
"""

from typing import Optional, Tuple

import numpy as np
import scipy.linalg

try:
    from .config import DEFAULT_CONFIG, SinglePendulumConfig
    from .dynamics import wrap_to_pi
except ImportError:
    from config import DEFAULT_CONFIG, SinglePendulumConfig
    from dynamics import wrap_to_pi


class LQRController:
    """
    LQR регулятор для одинарного перевернутого маятника на візку.
    Вектор стану: s = [x, theta, v, omega]
      x: координата візка (м)
      theta: відхилення від верхньої вертикалі (рад, 0 = строго вгору)
      v: лінійна швидкість візка (м/с)
      omega: кутова швидкість маятника (рад/с)
    """

    def __init__(self, cfg: SinglePendulumConfig = DEFAULT_CONFIG):
        self.cfg = cfg
        self.K = self.compute_gains()

    def compute_gains(self) -> np.ndarray:
        """
        Розрахунок матриці LQR K для лінеаризованої моделі біля верхньої точки (theta = 0).
        Враховує точний момент інерції I та центр мас l_com.
        Повертає вектор коефіцієнтів [k_x, k_th, k_v, k_om].
        """
        M = self.cfg.M
        m = self.cfg.m
        l = self.cfg.l
        I = self.cfg.I
        g = self.cfg.g

        # Детермінант матриці мас: D = I*(M+m) - m^2 * l^2
        D = I * (M + m) - (m * l) ** 2

        a32 = -((m * l) ** 2) * g / D
        a42 = (M + m) * m * g * l / D
        b3 = I / D
        b4 = -(m * l) / D

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
        Q = np.diag([self.cfg.q_x, self.cfg.q_theta, self.cfg.q_v, self.cfg.q_omega])
        R = np.array([[self.cfg.r_effort]])

        # Розв'язок неперервного алгебраїчного рівняння Ріккаті (ARE)
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        K = (np.linalg.inv(R) @ B.T @ P).flatten()
        return K

    def compute_force(
        self,
        state: np.ndarray,
        target_x: float = 0.0,
        custom_gains: Optional[Tuple[float, float, float, float]] = None,
    ) -> Tuple[float, float, bool]:
        """
        Обчислення керуючої сили LQR біля верхньої точки.
        Повертає: (u_sat, u_raw, is_saturated)
        """
        x, th, v, w = state
        th_err = wrap_to_pi(th)
        x_err = x - target_x

        if custom_gains is not None:
            k_th, k_om, k_x, k_v = custom_gains
            u_raw = float(-(k_x * x_err + k_th * th_err + k_v * v + k_om * w))
        else:
            # u = - K @ e = -(K[0]*x_err + K[1]*th_err + K[2]*v + K[3]*w)
            u_raw = float(
                -(
                    self.K[0] * x_err
                    + self.K[1] * th_err
                    + self.K[2] * v
                    + self.K[3] * w
                )
            )

        u_sat = float(np.clip(u_raw, -self.cfg.u_max, self.cfg.u_max))
        is_sat = bool(abs(u_sat - u_raw) > 1e-3)
        return u_sat, u_raw, is_sat
