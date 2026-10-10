"""
down_lqr_controller.py
======================
ООП реалізація Downward LQR (нижнього регулятора занулення та демпфування).
Синтезує оптимальний зворотний зв'язок K_down на основі лінеаризації
рівнянь подвійного маятника навколо нижньої точки спокою:
  x = 0, theta1 = pi, theta2 = pi, v = 0, omega1 = 0, omega2 = 0.

Активно переміщує візок для повного гасіння залишкових коливань ланок,
вирівнювання маятника вертикально вниз та центрування візка точно в x = 0.
"""

from typing import Tuple, Optional
import numpy as np
import scipy.linalg as la
try:
    from .config import SystemConfig, DEFAULT_CONFIG
    from .dynamics import wrap_to_pi
except ImportError:
    from config import SystemConfig, DEFAULT_CONFIG
    from dynamics import wrap_to_pi


class DownLQRController:
    """
    LQR-регулятор активного занулення та демпфування маятника в нижньому положенні.
    Забезпечує приведення системи до ідеального стартового стану:
      [x=0, theta1=pi, theta2=pi, v=0, w1=0, w2=0]
    для подальшого безпомилкового запуску розрахованого MPC Swing-Up.
    """

    def __init__(self, config: SystemConfig = DEFAULT_CONFIG):
        self.cfg = config
        self.M_down, self.D_down, self.K_down, self.Bu = self._build_linear_matrices()
        self.A, self.B = self._build_state_space()
        self.controllability_rank, self.is_controllable = self._check_controllability()
        self.P, self.K, self.closed_loop_poles = self._solve_riccati()

    def _build_linear_matrices(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Формує лінеаризовані матриці 2-го порядку навколо нижнього стану:
          theta1 = pi + alpha1, theta2 = pi + alpha2.
        Оскільки cos(pi) = -1, а sin(pi + alpha) = -sin(alpha) ≈ -alpha,
        гравітаційні моменти діють як відновлювальні (додатна жорсткість).
        """
        c = self.cfg

        # Лінеаризована матриця мас у нижній точці
        M_down = np.array([
            [c.h1,  -c.h2, -c.h3],
            [-c.h2,  c.h4,  c.h5],
            [-c.h3,  c.h5,  c.h6]
        ], dtype=np.float64)

        # Матриця в'язкого тертя та демпфування
        D_down = np.array([
            [c.b_cart, 0.0,         0.0],
            [0.0,      c.b_joint1,  0.0],
            [0.0,      0.0,         c.b_joint2]
        ], dtype=np.float64)

        # Матриця гравітаційної жорсткості (відновлювальні моменти нижньої ями)
        K_down = np.array([
            [0.0, 0.0,   0.0],
            [0.0, c.h7,  0.0],
            [0.0, 0.0,   c.h8]
        ], dtype=np.float64)

        # Вектор прикладання сили до візка
        Bu = np.array([[1.0], [0.0], [0.0]], dtype=np.float64)

        return M_down, D_down, K_down, Bu

    def _build_state_space(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Формує матриці простору станів dx/dt = A * x + B * u,
        де стан x = [x, alpha1, alpha2, v, w1, w2]^T.
        """
        inv_M = la.inv(self.M_down)

        A = np.block([
            [np.zeros((3, 3), dtype=np.float64), np.eye(3, dtype=np.float64)],
            [-inv_M @ self.K_down,               -inv_M @ self.D_down]
        ])

        B = np.block([
            [np.zeros((3, 1), dtype=np.float64)],
            [inv_M @ self.Bu]
        ])

        return A, B

    def _check_controllability(self) -> Tuple[int, bool]:
        """Перевіряє керованість за Калманом rank([B, AB, ..., A^5 B]) == 6."""
        n = self.A.shape[0]
        Co = np.hstack([np.linalg.matrix_power(self.A, i) @ self.B for i in range(n)])
        rank = int(np.linalg.matrix_rank(Co))
        return rank, bool(rank == n)

    def _solve_riccati(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Розв'язує рівняння Ріккаті (CARE) для синтезу оптимального занулюючого регулятора.
        """
        # Ваги для плавного демпфування та точного виходу на нуль
        Q_down = np.diag([
            30.0,   # Штраф за зміщення візка від центру (x)
            90.0,   # Штраф за відхилення ланки 1 від вертикалі вниз (alpha1)
            90.0,   # Штраф за відхилення ланки 2 від вертикалі вниз (alpha2)
            15.0,   # Штраф за лінійну швидкість візка (v)
            35.0,   # Штраф за кутову швидкість ланки 1 (w1)
            35.0    # Штраф за кутову швидкість ланки 2 (w2)
        ])
        R_down = np.array([[0.10]], dtype=np.float64)

        P = la.solve_continuous_are(self.A, self.B, Q_down, R_down)
        K = la.inv(R_down) @ self.B.T @ P

        closed_loop_A = self.A - self.B @ K
        poles = la.eigvals(closed_loop_A)

        return P, K.flatten(), poles

    def compute_force(
        self,
        current_state: np.ndarray,
        target_x: float = 0.0
    ) -> Tuple[float, float, bool]:
        """
        Обчислює стабілізуючу силу u_down = -K * e.
        Повертає: (u_clamped, u_raw, is_saturated).
        """
        x, th1, th2, v, w1, w2 = current_state

        # Кутові відхилення від положення "строго вниз" (pi):
        alpha1 = wrap_to_pi(th1 - np.pi)
        alpha2 = wrap_to_pi(th2 - np.pi)

        # Вектор похибки відносно цілі [target_x, 0, 0, 0, 0, 0]
        error = np.array([
            x - target_x,
            alpha1,
            alpha2,
            v,
            w1,
            w2
        ], dtype=np.float64)

        u_raw = float(-np.dot(self.K, error))

        # Насичення приводу
        f_max = self.cfg.F_max
        u_sat = float(np.clip(u_raw, -f_max, f_max))
        is_sat = bool(abs(u_raw) > f_max)

        return u_sat, u_raw, is_sat

    def is_zeroed(self, current_state: np.ndarray, target_x: float = 0.0) -> bool:
        """
        Перевіряє, чи система досягла ідеального нульового стану:
        - Візок біля target_x (|x - target_x| < 3 см)
        - Кути біля pi (|alpha| < 1.5°)
        - Швидкості практично нульові (|v| < 0.04 м/с, |w| < 5°/с)
        """
        x, th1, th2, v, w1, w2 = current_state
        alpha1_deg = np.degrees(abs(wrap_to_pi(th1 - np.pi)))
        alpha2_deg = np.degrees(abs(wrap_to_pi(th2 - np.pi)))
        w1_deg = np.degrees(abs(w1))
        w2_deg = np.degrees(abs(w2))

        return bool(
            abs(x - target_x) < 0.03 and
            alpha1_deg < 1.5 and
            alpha2_deg < 1.5 and
            abs(v) < 0.04 and
            w1_deg < 5.0 and
            w2_deg < 5.0
        )

