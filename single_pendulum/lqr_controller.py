"""
lqr_controller.py
=================
Аналітичний розрахунок оптимальних коефіцієнтів зворотного зв'язку LQR
(Linear Quadratic Regulator) для верхнього балансування та нижнього заспокоєння візка.
"""

from typing import Tuple
import numpy as np
import scipy.linalg

try:
    from .config import SinglePendulumConfig, DEFAULT_CONFIG
except ImportError:
    from config import SinglePendulumConfig, DEFAULT_CONFIG


class SinglePendulumLQR:
    """
    LQR регулятор для одинарного перевернутого маятника на візку.
    Вектор стану: s = [x, theta, v, omega]
    де theta = 0 — вертикальне положення вгору (Upright),
       theta = pi — нижнє висяче положення (Down).
    """

    def __init__(self, cfg: SinglePendulumConfig = DEFAULT_CONFIG):
        self.cfg = cfg
        self.K_top = self.compute_top_lqr()
        self.K_down = self.compute_down_lqr()

    def compute_top_lqr(self) -> np.ndarray:
        """
        Розрахунок матриці LQR K для лінеаризованої моделі біля верхньої точки (theta = 0).
        Повертає вектор коефіцієнтів [k_x, k_th, k_v, k_om].
        """
        M = self.cfg.M
        m = self.cfg.m
        l = self.cfg.l
        g = self.cfg.g

        denom = l * (4.0 * M + m)
        a32 = -3.0 * m * g / (4.0 * M + m)
        a42 = 3.0 * (M + m) * g / denom
        b3 = 4.0 / (4.0 * M + m)
        b4 = -3.0 / denom

        A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, a32, 0.0, 0.0],
            [0.0, a42, 0.0, 0.0]
        ], dtype=float)

        B = np.array([[0.0], [0.0], [b3], [b4]], dtype=float)
        Q = np.diag([self.cfg.q_x, self.cfg.q_theta, self.cfg.q_v, self.cfg.q_omega])
        R = np.array([[self.cfg.r_effort]])

        # Розв'язок неперервного алгебраїчного рівняння Ріккаті (ARE)
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        K = (np.linalg.inv(R) @ B.T @ P).flatten()
        return K

    def compute_down_lqr(self) -> np.ndarray:
        """
        LQR для нижньої точки (theta = pi): демпфування коливань маятника
        та активне повернення візка в нуль (x = 0).
        """
        M = self.cfg.M
        m = self.cfg.m
        l = self.cfg.l
        g = self.cfg.g

        denom = l * (4.0 * M + m)
        # Біля нижньої точки знак гравітаційного члена для dtheta змінюється на протилежний
        a32 = 3.0 * m * g / (4.0 * M + m)
        a42 = -3.0 * (M + m) * g / denom
        b3 = 4.0 / (4.0 * M + m)
        b4 = 3.0 / denom

        A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, a32, 0.0, 0.0],
            [0.0, a42, 0.0, 0.0]
        ], dtype=float)

        B = np.array([[0.0], [0.0], [b3], [b4]], dtype=float)
        Q = np.diag([10.0, 40.0, 5.0, 15.0])
        R = np.array([[0.1]])

        try:
            P = scipy.linalg.solve_continuous_are(A, B, Q, R)
            K = (np.linalg.inv(R) @ B.T @ P).flatten()
        except Exception:
            # Резервні надійні коефіцієнти PD
            K = np.array([12.0, 25.0, 10.0, 6.0])
        return K

    def compute_force(self, state: np.ndarray, target_x: float = 0.0,
                      custom_gains: Tuple[float, float, float, float] = None) -> float:
        """
        Обчислення керуючої сили LQR біля верхньої точки.
        state: [x, theta, v, omega]
        custom_gains: опційні ручні коефіцієнти (k_theta, k_omega, k_x, k_v) зі слайдерів.
        """
        x, th, v, w = state
        # Нормалізація кута до [-pi, pi] відносно верхньої точки (0 = вертикаль вгору)
        th_err = (th + np.pi) % (2.0 * np.pi) - np.pi
        x_err = x - target_x

        if custom_gains is not None:
            k_th, k_om, k_x, k_v = custom_gains
            # u = - (k_x * x + k_th * th + k_v * v + k_om * w)
            # У конвенції sim_pygame знак уже враховано в коефіцієнтах:
            u = k_th * th_err + k_om * w + k_x * x_err + k_v * v
        else:
            # u = - K @ e = -(K[0]*x_err + K[1]*th_err + K[2]*v + K[3]*w)
            u = -(self.K_top[0] * x_err + self.K_top[1] * th_err + self.K_top[2] * v + self.K_top[3] * w)

        return float(u)
