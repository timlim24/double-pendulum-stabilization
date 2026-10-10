"""
lqr_controller.py
=================
ООП реалізація LQR (Linear Quadratic Regulator) для стабілізації
подвійного перевернутого маятника у верхній нестійкій точці рівноваги.
Синтезує оптимальний вектор зворотного зв'язку K на основі лінеаризованої
системи в просторі станів та рівняння Ріккаті (CARE).
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


class LQRController:
    """
    Контролер оптимального лінійно-квадратичного регулятора (LQR).
    Формує закон керування: u = -K * (state - ref_state), насичений за силі.
    """

    def __init__(self, config: SystemConfig = DEFAULT_CONFIG):
        self.cfg = config
        self.M0, self.D0, self.K0, self.Bu = self._build_linear_matrices()
        self.A, self.B = self._build_state_space()
        self.controllability_rank, self.is_controllable = self._check_controllability()
        self.P, self.K, self.closed_loop_poles = self._solve_riccati()

    def _build_linear_matrices(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Формує лінеаризовані матриці другого порядку:
        M0 * ddq + D0 * dq + K0 * q = Bu * u
        навколо вертикального стану theta1 = 0, theta2 = 0.
        """
        c = self.cfg

        # Лінеаризована узагальнена матриця інерції
        M0 = np.array([
            [c.h1, c.h2, c.h3],
            [c.h2, c.h4, c.h5],
            [c.h3, c.h5, c.h6]
        ], dtype=np.float64)

        # Матриця в'язкого тертя та демпфування
        D0 = np.array([
            [c.b_cart, 0.0,         0.0],
            [0.0,      c.b_joint1,  0.0],
            [0.0,      0.0,         c.b_joint2]
        ], dtype=np.float64)

        # Матриця гравітаційного опрокидувального моменту (жорсткості)
        K0 = np.array([
            [0.0,  0.0,   0.0],
            [0.0, -c.h7,  0.0],
            [0.0,  0.0,  -c.h8]
        ], dtype=np.float64)

        # Вектор прикладання сили
        Bu = np.array([[1.0], [0.0], [0.0]], dtype=np.float64)

        return M0, D0, K0, Bu

    def _build_state_space(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Переводить систему 2-го порядку в 6x6 простір станів 1-го порядку:
        dx/dt = A * x + B * u, де x = [x, theta1, theta2, v, omega1, omega2]^T
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

    def _check_controllability(self) -> Tuple[int, bool]:
        """Перевіряє ранг матриці керованості Калмана rank([B, AB, ..., A^5 B]) == 6."""
        n = self.A.shape[0]
        Co = np.hstack([np.linalg.matrix_power(self.A, i) @ self.B for i in range(n)])
        rank = int(np.linalg.matrix_rank(Co))
        return rank, bool(rank == n)

    def _solve_riccati(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Розв'язує неперервне алгебраїчне рівняння Ріккаті (CARE):
        A^T * P + P * A - P * B * R^-1 * B^T * P + Q = 0
        та розраховує коефіцієнти зворотного зв'язку K = R^-1 * B^T * P.
        """
        Q = self.cfg.Q
        R = self.cfg.R

        P = la.solve_continuous_are(self.A, self.B, Q, R)
        K = (la.inv(R) @ self.B.T @ P).flatten()

        # Полюси замкненої системи (A - B * K)
        cl_A = self.A - self.B @ K.reshape(1, -1)
        cl_poles = la.eigvals(cl_A)

        return P, K, cl_poles

    def update_gains(self, q_th: Optional[float] = None, q_pos: Optional[float] = None) -> None:
        """Динамічне оновлення коефіцієнтів Q під час роботи інтерфейсу."""
        if q_th is not None:
            self.cfg.q_th1 = q_th * 0.8
            self.cfg.q_th2 = q_th
        if q_pos is not None:
            self.cfg.q_x = q_pos

        self.P, self.K, self.closed_loop_poles = self._solve_riccati()

    def compute_force(
        self,
        state: np.ndarray,
        target_x: float = 0.0
    ) -> Tuple[float, float, bool]:
        """
        Обчислює керуючу силу:
        u = -K * e, де e = [x - x_target, wrap(th1), wrap(th2), v, w1, w2]^T.
        
        Повертає:
          (u_saturated, u_raw, is_saturated)
        """
        x, th1, th2, v, w1, w2 = state

        # Кутові помилки рахуються відносно верхньої точки (0.0 рад)
        th1_err = wrap_to_pi(th1)
        th2_err = wrap_to_pi(th2)

        err_vec = np.array([
            x - target_x,
            th1_err,
            th2_err,
            v,
            w1,
            w2
        ], dtype=np.float64)

        u_raw = float(-np.dot(self.K, err_vec))
        f_max = self.cfg.F_max
        u_sat = float(np.clip(u_raw, -f_max, f_max))
        is_sat = bool(abs(u_raw) > f_max)

        return u_sat, u_raw, is_sat
