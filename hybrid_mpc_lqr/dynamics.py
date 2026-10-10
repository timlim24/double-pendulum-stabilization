"""
dynamics.py
===========
ООП фізичний рушій подвійного інвертованого маятника на рухомому візку.
Реалізує повні нелінійні рівняння руху Ейлера-Лагранжа, чисельне інтегрування
методом Рунге-Кутти 4-го порядку (RK4), взаємодію з відбійниками та курсором миші.
"""

from typing import Tuple, Dict, Any, Optional
import numpy as np

try:
    from .config import SystemConfig, DEFAULT_CONFIG
except ImportError:
    from config import SystemConfig, DEFAULT_CONFIG


def wrap_to_pi(angle: float) -> float:
    """Нормалізує кут до діапазону [-pi, pi], де 0 — верхнє вертикальне положення."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


class DoublePendulumDynamics:
    """
    Клас фізичної динаміки подвійного маятника на візку.
    Вектор стану системи: [x, theta1, theta2, v, omega1, omega2]
      x: координата візка (м)
      theta1: кут відхилення першої ланки (рад), 0 = вгору, pi = вниз
      theta2: кут відхилення другої ланки (рад), 0 = вгору, pi = вниз
      v: лінійна швидкість візка (м/с)
      omega1: кутова швидкість ланки 1 (рад/с)
      omega2: кутова швидкість ланки 2 (рад/с)
    """

    def __init__(self, config: SystemConfig = DEFAULT_CONFIG):
        self.cfg = config

        # Початковий стан: маятник опущений униз (0.99*pi як у моделі MPC)
        self.state = np.array([0.0, 0.99 * np.pi, 0.99 * np.pi, 0.0, 0.0, 0.0], dtype=np.float64)
        self.time: float = 0.0
        self.step_count: int = 0

        # Останні телеметричні показники
        self.last_u: float = 0.0
        self.last_a: float = 0.0
        self.last_ext_force: float = 0.0
        self.is_saturated: bool = False
        self.track_collision: bool = False

    def reset(self, state: Optional[np.ndarray] = None, hanging_down: bool = True) -> None:
        """Скидає стан системи у задане або нижнє положення."""
        if state is not None:
            self.state = np.array(state, dtype=np.float64).copy()
        elif hanging_down:
            # Нижній спокійний стан (0.99*pi)
            self.state = np.array([0.0, 0.99 * np.pi, 0.99 * np.pi, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            # Верхній вертикальний стан
            self.state = np.zeros(6, dtype=np.float64)

        self.time = 0.0
        self.step_count = 0
        self.last_u = 0.0
        self.last_a = 0.0
        self.last_ext_force = 0.0
        self.is_saturated = False
        self.track_collision = False

    def get_derivatives(self, s: np.ndarray, u_force: float, ext_cart_force: float = 0.0) -> np.ndarray:
        """
        Обчислює похідні ds/dt = [v, omega1, omega2, ddot_x, ddot_theta1, ddot_theta2]
        з нелінійної системи рівнянь: M(q) * ddot_q = F_rhs(q, dq, u)
        """
        c = self.cfg
        x, th1, th2, v, w1, w2 = s

        # Тригонометричні коефіцієнти
        c1 = np.cos(th1)
        s1 = np.sin(th1)
        c2 = np.cos(th2)
        s2 = np.sin(th2)
        c12 = np.cos(th1 - th2)
        s12 = np.sin(th1 - th2)

        # Нелінійна матриця мас M(q)
        M = np.array([
            [c.h1,       c.h2 * c1,   c.h3 * c2],
            [c.h2 * c1,  c.h4,        c.h5 * c12],
            [c.h3 * c2,  c.h5 * c12,  c.h6]
        ], dtype=np.float64)

        # Сумарна поздовжня сила на візок
        total_cart_force = u_force + ext_cart_force - c.b_cart * v

        # Вектор правих частин узагальнених сил F_rhs(q, dq, u)
        F_cart = (
            total_cart_force
            + c.h2 * (w1**2) * s1
            + c.h3 * (w2**2) * s2
        )

        F_link1 = (
            c.h7 * s1
            - c.h5 * (w2**2) * s12
            - c.b_joint1 * w1
        )

        F_link2 = (
            c.h8 * s2
            + c.h5 * (w1**2) * s12
            - c.b_joint2 * w2
        )

        F_rhs = np.array([F_cart, F_link1, F_link2], dtype=np.float64)

        # Розв'язуємо лінійну систему для других похідних: M * ddot_q = F_rhs
        try:
            ddot_q = np.linalg.solve(M, F_rhs)
        except np.linalg.LinAlgError:
            ddot_q = np.linalg.lstsq(M, F_rhs, rcond=None)[0]

        return np.array([v, w1, w2, ddot_q[0], ddot_q[1], ddot_q[2]], dtype=np.float64)

    def step(self, u_control: float, ext_force: float = 0.0, dt: Optional[float] = None) -> np.ndarray:
        """
        Здійснює один крок числового інтегрування методом Рунге-Кутти 4-го порядку (RK4).
        Обмежує керуючу силу за F_max та враховує пружні відбійники рейки.
        """
        c = self.cfg
        step_dt = c.sim_dt if dt is None else dt

        # Насичення приводу
        u_sat = float(np.clip(u_control, -c.F_max, c.F_max))
        self.last_u = u_sat
        self.last_ext_force = ext_force
        self.is_saturated = bool(abs(u_control) > c.F_max)

        # Інтегрування RK4
        k1 = self.get_derivatives(self.state, u_sat, ext_force)
        k2 = self.get_derivatives(self.state + 0.5 * step_dt * k1, u_sat, ext_force)
        k3 = self.get_derivatives(self.state + 0.5 * step_dt * k2, u_sat, ext_force)
        k4 = self.get_derivatives(self.state + step_dt * k3, u_sat, ext_force)

        self.state += (step_dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        self.last_a = float(k1[3])

        # Обробка зіткнення з відбійниками на межах рейки [-X_rail, +X_rail]
        self._handle_rail_boundaries()

        self.time += step_dt
        self.step_count += 1

        return self.state

    def _handle_rail_boundaries(self) -> None:
        """Реагування на досягнення кінців рейки за допомогою пружно-демпферного бар'єра."""
        c = self.cfg
        x = self.state[0]
        v = self.state[3]
        self.track_collision = False

        if x > c.X_rail:
            excess = x - c.X_rail
            self.state[0] = c.X_rail
            # Відскок із затуханням
            if v > 0.0:
                self.state[3] = -0.2 * v
            self.track_collision = True
        elif x < -c.X_rail:
            excess = -c.X_rail - x
            self.state[0] = -c.X_rail
            if v < 0.0:
                self.state[3] = -0.2 * v
            self.track_collision = True

    def get_cartesian_points(self) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        """
        Повертає декартові координати в метрах:
        - (x_cart, y_cart)
        - (x_joint, y_joint): кінчик ланки 1 / шарнір 2
        - (x_tip, y_tip): кінчик ланки 2
        У декартовій системі y = 0 — рівень візка, y > 0 — вгору.
        """
        c = self.cfg
        x = self.state[0]
        th1 = self.state[1]
        th2 = self.state[2]

        # Візок
        cart_pt = (x, 0.0)

        # Кінець ланки 1
        x1 = x + c.L1 * np.sin(th1)
        y1 = c.L1 * np.cos(th1)
        joint_pt = (x1, y1)

        # Кінець ланки 2
        x2 = x1 + c.L2 * np.sin(th2)
        y2 = y1 + c.L2 * np.cos(th2)
        tip_pt = (x2, y2)

        return cart_pt, joint_pt, tip_pt

    def get_total_energy(self) -> Tuple[float, float, float]:
        """
        Обчислює повну механічну енергію: E_kin, E_pot, E_total.
        У верхній вертикальній точці E_pot максимальна.
        """
        c = self.cfg
        x, th1, th2, v, w1, w2 = self.state

        # Кінетична енергія візка
        e_k_cart = 0.5 * c.m0 * (v**2)

        # Кінетична енергія ланки 1
        v1_sq = (v + c.l1 * w1 * np.cos(th1))**2 + (c.l1 * w1 * np.sin(th1))**2
        e_k_link1 = 0.5 * c.m1 * v1_sq + 0.5 * c.J1 * (w1**2)

        # Кінетична енергія ланки 2
        v2_sq = (
            (v + c.L1 * w1 * np.cos(th1) + c.l2 * w2 * np.cos(th2))**2 +
            (c.L1 * w1 * np.sin(th1) + c.l2 * w2 * np.sin(th2))**2
        )
        e_k_link2 = 0.5 * c.m2 * v2_sq + 0.5 * c.J2 * (w2**2)

        e_kin = e_k_cart + e_k_link1 + e_k_link2

        # Потенціальна енергія (відносно шарніра на візку y=0)
        e_pot = c.m1 * c.g * c.l1 * np.cos(th1) + c.m2 * c.g * (c.L1 * np.cos(th1) + c.l2 * np.cos(th2))

        return float(e_kin), float(e_pot), float(e_kin + e_pot)

    def is_near_upright(self, max_angle_rad: float, max_omega_rad_s: float) -> bool:
        """Перевіряє, чи перебуває система поблизу вертикальної вершини."""
        th1_w = abs(wrap_to_pi(self.state[1]))
        th2_w = abs(wrap_to_pi(self.state[2]))
        w1 = abs(self.state[4])
        w2 = abs(self.state[5])

        return bool(th1_w < max_angle_rad and th2_w < max_angle_rad and
                    w1 < max_omega_rad_s and w2 < max_omega_rad_s)

    def get_telemetry(self) -> Dict[str, Any]:
        """Формує словник поточних показників системи для інтерфейсу."""
        x, th1, th2, v, w1, w2 = self.state
        th1_w = wrap_to_pi(th1)
        th2_w = wrap_to_pi(th2)
        e_kin, e_pot, e_tot = self.get_total_energy()
        cart_pt, joint_pt, tip_pt = self.get_cartesian_points()

        return {
            "time": self.time,
            "step": self.step_count,
            "x": float(x),
            "v": float(v),
            "theta1_rad": float(th1),
            "theta2_rad": float(th2),
            "theta1_wrapped_rad": float(th1_w),
            "theta2_wrapped_rad": float(th2_w),
            "theta1_deg": float(np.degrees(th1_w)),
            "theta2_deg": float(np.degrees(th2_w)),
            "omega1_deg_s": float(np.degrees(w1)),
            "omega2_deg_s": float(np.degrees(w2)),
            "u": float(self.last_u),
            "a": float(self.last_a),
            "ext_force": float(self.last_ext_force),
            "is_saturated": self.is_saturated,
            "track_collision": self.track_collision,
            "e_kin": e_kin,
            "e_pot": e_pot,
            "e_tot": e_tot,
            "cart_pt": cart_pt,
            "joint_pt": joint_pt,
            "tip_pt": tip_pt
        }
