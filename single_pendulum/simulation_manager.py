"""
simulation_manager.py
=====================
Менеджер симуляції та скінченний автомат станів (FSM) системи одинарного маятника на візку.
Координує взаємодію між чистою фізичною динамікою, розгоном візка за MPC та стабілізатором LQR:
  - LQR активний ЗАВЖДИ (Omnipresent Catch), щойно маятник опиняється у верхній зоні (|θ| < 20°),
    перериваючи будь-який інший стан (гасіння, розгін, очікування).
  - MPC подає силу виключно на візок (без телепортації стану чи кутів).
  - Маятник рухається суто за законами нелінійної фізики RK4 (пасивний шарнір з енкодером).
"""

from enum import Enum
from typing import Optional

import numpy as np

try:
    from .config import DEFAULT_CONFIG, SinglePendulumConfig
    from .down_lqr_controller import DownLQRController
    from .dynamics import SinglePendulumDynamics, wrap_to_pi
    from .lqr_controller import LQRController
    from .mpc_controller import MPCTrajectoryController
except ImportError:
    from config import DEFAULT_CONFIG, SinglePendulumConfig
    from down_lqr_controller import DownLQRController
    from dynamics import SinglePendulumDynamics, wrap_to_pi
    from lqr_controller import LQRController
    from mpc_controller import MPCTrajectoryController


class ControlMode(Enum):
    FALLEN = "Вільний рух / Відкат"
    LQR_DOWN_ZERO = "Занулення вниз (Down-LQR)"
    WAITING_FOR_TRIGGER = "Очікує запуску MPC (Клавіша S)"
    MPC_SWINGUP = "Розгін візка (MPC Swing-Up)"
    LQR_BALANCE = "Балансування (LQR)"
    MANUAL_OFF = "Мотор вимкнено"


class SimulationManager:
    """
    Головний координатор гібридного керування одинарним маятником.
    Реалізує незалежне включення LQR, чисту фізику візка і маятника та телеметрію.
    """

    def __init__(self, config: SinglePendulumConfig = DEFAULT_CONFIG):
        self.cfg = config
        self.dynamics = SinglePendulumDynamics(config)
        self.lqr = LQRController(config)
        self.down_lqr = DownLQRController(config)
        self.mpc = MPCTrajectoryController(config)

        # Початковий режим: активне заспокоєння коливань внизу
        self.mode = ControlMode.LQR_DOWN_ZERO
        self.target_cart_x: float = 0.0

        # Зовнішня сила взаємодії (миша, клавіатура)
        self.ext_force: float = 0.0
        self.status_message: str = "✓ Заспокоєння коливань внизу (Down-LQR)..."

    def reset(self, start_down: bool = True) -> None:
        """Скидає симуляцію у початковий стан."""
        self.mpc.stop()
        if start_down:
            self.dynamics.reset(
                state=np.array([0.0, np.pi, 0.0, 0.0], dtype=np.float64)
            )
            self.mode = ControlMode.LQR_DOWN_ZERO
            self.status_message = "✓ Маятник у нижньому стані. Заспокоєння коливань..."
        else:
            self.dynamics.reset(hanging_down=False)
            self.mode = ControlMode.LQR_BALANCE
            self.target_cart_x = self.dynamics.state[0]
            self.status_message = "Вертикальна рівновага: LQR активний"
        self.ext_force = 0.0

    def trigger_down_zero(self) -> None:
        """
        Активує нижній Down-LQR алгоритм:
        гасить залишкові коливання маятника та центрує візок точно в x = 0.
        """
        self.mpc.stop()
        self.mode = ControlMode.LQR_DOWN_ZERO
        self.status_message = (
            "Активне занулення (Down-LQR): центрування візка в 0 та гасіння коливань..."
        )

    def trigger_mpc_swingup(self) -> bool:
        """
        Запускає подачу сили розгону візка за траєкторією MPC.
        КРИТИЧНО: ЖОДНОЇ ТЕЛЕПОРТАЦІЇ! Поточний стан маятника та координата візка
        залишаються рівно такими, якими вони є в момент виклику.
        """
        if not self.mpc.loaded:
            self.status_message = (
                "Помилка: файл траєкторії MPC не знайдено! Запустіть mpc_generator.py"
            )
            return False

        self.mpc.start()
        self.mode = ControlMode.MPC_SWINGUP
        self.status_message = "▶ Виконується оптимальний розгін візка MPC..."
        return True

    def toggle_motor(self) -> None:
        """Вмикає або вимикає привід візка."""
        if self.mode == ControlMode.MANUAL_OFF:
            th_w = abs(wrap_to_pi(self.dynamics.state[1]))
            catch_rad = np.radians(self.cfg.catch_angle_deg)
            if th_w < catch_rad:
                self.mode = ControlMode.LQR_BALANCE
                self.target_cart_x = self.dynamics.state[0]
                self.status_message = "Мотор увімкнено: LQR підхопив баланс"
            else:
                self.mode = ControlMode.FALLEN
                self.status_message = "Мотор увімкнено: плавне повернення в центр"
        else:
            self.mpc.stop()
            self.mode = ControlMode.MANUAL_OFF
            self.status_message = "Мотор вимкнено (вільне хитання)"

    def apply_kick(self, direction: float, speed_step: Optional[float] = None) -> None:
        """Миттєвий ударний імпульс швидкості (Kick) для збурення візка."""
        step = speed_step if speed_step is not None else self.cfg.nudge_kick_speed
        self.dynamics.state[2] += direction * step
        self.status_message = (
            f"⚡ Поштовх візка: імпульс швидкості Δv = {direction * step:+.2f} м/с"
        )

    def set_continuous_nudge(self, force: float) -> None:
        """Задає постійну горизонтальну тягу при утриманні клавіші."""
        self.ext_force = force

    def set_mouse_drag_target(self, target_x: float) -> None:
        """Розраховує пружинно-демпферну силу тяги від курсора миші на візок."""
        cart_x = self.dynamics.state[0]
        cart_v = self.dynamics.state[2]
        f_spring = self.cfg.k_drag * (target_x - cart_x) - self.cfg.d_drag * cart_v
        self.ext_force = float(f_spring)

    def release_mouse_drag(self) -> None:
        """Знімає силу перетягування мишкою."""
        self.ext_force = 0.0
        if self.mode == ControlMode.LQR_BALANCE:
            self.target_cart_x = float(self.dynamics.state[0])

    def update(self, dt: float) -> None:
        """
        Один крок симуляції та фізичної динаміки RK4.
        """
        # =====================================================================
        # 1. НЕЗАЛЕЖНИЙ ОМНІПРЕЗЕНТНИЙ МОНІТОРИНГ LQR:
        # У БУДЬ-ЯКИЙ МОМЕНТ, якщо маятник опинився біля вертикальної вершини
        # (піднятий розгоном MPC, підкинутий мишкою чи розгойданий вручну),
        # LQR МИТТЄВО перехоплює контроль та стабілізує систему!
        # =====================================================================
        catch_rad = np.radians(self.cfg.catch_angle_deg)
        th_w = abs(wrap_to_pi(self.dynamics.state[1]))
        w_abs = abs(self.dynamics.state[3])

        is_upright = bool(th_w < catch_rad and w_abs < self.cfg.catch_max_omega)

        if is_upright and self.mode != ControlMode.MANUAL_OFF:
            if self.mode != ControlMode.LQR_BALANCE:
                if self.mode == ControlMode.MPC_SWINGUP:
                    self.mpc.stop()
                self.mode = ControlMode.LQR_BALANCE
                self.target_cart_x = self.dynamics.state[0]
                self.status_message = (
                    "⚡ LQR підхопив керування! Балансування активовано"
                )

        # =====================================================================
        # 2. ВИЗНАЧЕННЯ СИЛИ МОТОРА ВІЗКА ЗА РЕЖИМОМ КЕРУВАННЯ
        # =====================================================================
        u_control = 0.0

        if self.mode == ControlMode.LQR_BALANCE:
            u_sat, u_raw, is_sat = self.lqr.compute_force(
                self.dynamics.state, target_x=self.target_cart_x
            )
            u_control = u_sat

            # Якщо збурення занадто велике і маятник вилетів із зони балансу:
            fall_rad = np.radians(self.cfg.fall_angle_deg)
            if th_w > fall_rad:
                self.mode = ControlMode.FALLEN
                self.status_message = "⚠ Маятник упав! Повернення візка в центр..."

        elif self.mode == ControlMode.LQR_DOWN_ZERO:
            u_sat, u_raw, is_sat = self.down_lqr.compute_force(
                self.dynamics.state, target_x=0.0
            )
            u_control = u_sat

            if self.down_lqr.is_zeroed(self.dynamics.state, target_x=0.0):
                self.mode = ControlMode.WAITING_FOR_TRIGGER
                self.status_message = (
                    "✓ Нуль досягнуто! Натисніть [S] на клавіатурі або [▶ MPC Swing-Up]"
                )

        elif self.mode == ControlMode.WAITING_FOR_TRIGGER:
            # Утримання візка в нулі (м'який PD контур на x = 0)
            x_val = self.dynamics.state[0]
            v_val = self.dynamics.state[2]
            u_control = -18.0 * x_val - 6.0 * v_val

            # Якщо маятник збурили ззовні — повернутися до гасіння
            if not self.down_lqr.is_zeroed(self.dynamics.state, target_x=0.0):
                self.mode = ControlMode.LQR_DOWN_ZERO
                self.status_message = "Заспокоєння коливань внизу..."

        elif self.mode == ControlMode.MPC_SWINGUP:
            u_mpc, should_handover, is_finished = self.mpc.step(dt, self.dynamics.state)
            u_control = u_mpc

            if should_handover:
                self.mode = ControlMode.LQR_BALANCE
                self.target_cart_x = self.dynamics.state[0]
                self.status_message = "⚡ LQR підхопив на вершині! Баланс активний"
            elif is_finished:
                self.mode = ControlMode.FALLEN
                self.status_message = (
                    "MPC завершився без виходу у верхню точку. Повернення в центр..."
                )

        elif self.mode == ControlMode.FALLEN:
            # Плавне повернення візка в центр x = 0
            cart_x = self.dynamics.state[0]
            cart_v = self.dynamics.state[2]

            if abs(cart_x) < 0.015 and abs(cart_v) < 0.05:
                self.mode = ControlMode.LQR_DOWN_ZERO
                self.status_message = "Візок у центрі. Заспокоєння коливань внизу..."
            else:
                # Плавна сила центрування
                u_control = -14.0 * cart_x - 5.0 * cart_v

        # =====================================================================
        # 3. ФІЗИЧНИЙ КРОК ЧИСЛОВОГО ІНТЕГРУВАННЯ RK4
        # =====================================================================
        if self.mode == ControlMode.MANUAL_OFF:
            u_control = 0.0

        self.dynamics.rk4_step(dt, u_cart=u_control, ext_force=self.ext_force)
