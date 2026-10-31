"""
dynamics.py
===========
Фізичний рушій одинарного інвертованого маятника на рухомому візку.
Реалізує повні нелінійні рівняння руху Ейлера-Лагранжа, чисельне інтегрування
методом Рунге-Кутти 4-го порядку (RK4), взаємодію з механічними відбійниками та курсором миші.
"""

from typing import Optional, Tuple

import numpy as np

try:
    from .config import DEFAULT_CONFIG, SinglePendulumConfig
except ImportError:
    from config import DEFAULT_CONFIG, SinglePendulumConfig


def wrap_to_pi(angle: float) -> float:
    """Нормалізує кут до діапазону [-pi, pi], де 0 — верхнє вертикальне положення."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


class SinglePendulumDynamics:
    """
    Клас фізичної динаміки одинарного маятника на візку.
    Вектор стану системи: [x, theta, v, omega]
      x: координата візка (м), 0 = центр рейки
      theta: кут відхилення маятника (рад), 0 = вгору, ±pi = вниз
      v: лінійна швидкість візка (м/с)
      omega: кутова швидкість маятника (рад/с)
    """

    def __init__(self, config: SinglePendulumConfig = DEFAULT_CONFIG):
        self.cfg = config

        # Початковий стан: маятник висить спокійно внизу (theta = pi)
        self.state = np.array([0.0, np.pi, 0.0, 0.0], dtype=np.float64)
        self.time: float = 0.0
        self.step_count: int = 0

        # Телеметрія
        self.last_u: float = 0.0
        self.last_a: float = 0.0
        self.last_ext_force: float = 0.0
        self.is_saturated: bool = False
        self.track_collision: bool = False

    def reset(
        self, state: Optional[np.ndarray] = None, hanging_down: bool = True
    ) -> None:
        """Скидає стан системи у задане або нижнє спокійне положення."""
        if state is not None:
            self.state = np.array(state, dtype=np.float64).copy()
        elif hanging_down:
            self.state = np.array([0.0, np.pi, 0.0, 0.0], dtype=np.float64)
        else:
            self.state = np.zeros(4, dtype=np.float64)

        self.time = 0.0
        self.step_count = 0
        self.last_u = 0.0
        self.last_a = 0.0
        self.last_ext_force = 0.0
        self.is_saturated = False
        self.track_collision = False

    def compute_energy(
        self, state: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float]:
        """
        Обчислює кінетичну, потенційну та повну енергію маятника відносно верхньої точки.
        На вершині: E = 0 (E_kin = 0, E_pot = 0).
        Внизу: E = -2 * m * g * l.
        """
        st = state if state is not None else self.state
        th_w = wrap_to_pi(st[1])
        w = st[3]

        mgl = self.cfg.m * self.cfg.g * self.cfg.l
        e_kin = 0.5 * self.cfg.I * (w**2)
        e_pot = mgl * (np.cos(th_w) - 1.0)
        return float(e_kin), float(e_pot), float(e_kin + e_pot)

    def equations_of_motion(
        self, st: np.ndarray, total_force: float
    ) -> Tuple[np.ndarray, float]:
        """
        Нелінійні рівняння руху системи (візок + маятник):
        Повертає похідну стану [v, omega, x_acc, theta_acc] та прискорення візка x_acc.
        """
        x_val, th_val, v_val, w_val = st[0], st[1], st[2], st[3]

        M_val = self.cfg.M
        m_val = self.cfg.m
        l_val = self.cfg.l
        I_val = self.cfg.I
        g_val = self.cfg.g
        b_c = self.cfg.b_cart
        b_p = self.cfg.b_pend

        sin_t = np.sin(th_val)
        cos_t = np.cos(th_val)

        # Рівняння руху:
        # (M + m) * x_acc + m * l * theta_acc * cos(th) - m * l * omega^2 * sin(th) = total_force - b_c * v
        # I * theta_acc + m * l * x_acc * cos(th) - m * g * l * sin(th) = - b_p * omega
        temp = (total_force - b_c * v_val + m_val * l_val * (w_val**2) * sin_t) / (
            M_val + m_val
        )
        denom = (I_val / (m_val * l_val)) - (m_val * l_val * (cos_t**2)) / (
            M_val + m_val
        )

        theta_acc = (
            g_val * sin_t - cos_t * temp - (b_p / (m_val * l_val)) * w_val
        ) / denom
        x_acc = temp - (m_val * l_val * theta_acc * cos_t) / (M_val + m_val)

        # Фізичне обмеження прискорення та сили від крокового двигуна
        a_max = self.cfg.a_max
        if abs(x_acc) > a_max:
            x_acc = float(np.clip(x_acc, -a_max, a_max))
            theta_acc = (
                g_val * sin_t - x_acc * cos_t - (b_p / (m_val * l_val)) * w_val
            ) / (I_val / (m_val * l_val))

        # Пружний удар об фізичні стопери на краях рейки (±0.2945 м)
        x_lim = self.cfg.x_limit_physical
        if x_val >= x_lim:
            f_bump = -self.cfg.k_bumper * (x_val - x_lim) - self.cfg.d_bumper * max(
                0.0, v_val
            )
            x_acc += f_bump / M_val
            self.track_collision = True
        elif x_val <= -x_lim:
            f_bump = -self.cfg.k_bumper * (x_val + x_lim) - self.cfg.d_bumper * min(
                0.0, v_val
            )
            x_acc += f_bump / M_val
            self.track_collision = True
        else:
            self.track_collision = False

        deriv = np.array([v_val, w_val, x_acc, theta_acc], dtype=np.float64)
        return deriv, x_acc

    def rk4_step(self, dt: float, u_cart: float = 0.0, ext_force: float = 0.0) -> None:
        """
        Один крок числового інтегрування методом Рунге-Кутти 4-го порядку (RK4).
        """
        # Обмеження сили мотора візка
        u_sat = float(np.clip(u_cart, -self.cfg.u_max, self.cfg.u_max))
        self.is_saturated = bool(abs(u_sat - u_cart) > 1e-4)

        # Обмеження проти-ЕРС при перевищенні максимальної швидкості приводу
        if self.state[2] >= self.cfg.v_max and u_sat > 0:
            u_sat = 0.0
        elif self.state[2] <= -self.cfg.v_max and u_sat < 0:
            u_sat = 0.0

        total_force = u_sat + ext_force
        self.last_u = u_sat
        self.last_ext_force = ext_force

        st = self.state
        k1, a1 = self.equations_of_motion(st, total_force)
        k2, _ = self.equations_of_motion(st + 0.5 * dt * k1, total_force)
        k3, _ = self.equations_of_motion(st + 0.5 * dt * k2, total_force)
        k4, _ = self.equations_of_motion(st + dt * k3, total_force)

        self.state = st + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        self.last_a = a1

        # Обмежуємо швидкість візка
        self.state[2] = float(np.clip(self.state[2], -self.cfg.v_max, self.cfg.v_max))

        self.time += dt
        self.step_count += 1
