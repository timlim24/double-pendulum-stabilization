"""
simulation_manager.py
=====================
Менеджер симуляції та скінченний автомат станів (FSM) гібридної системи.
Координує взаємодію між чистою фізичною динамікою, розгоном візка за MPC та стабілізатором LQR:
  - LQR активний ЗАВЖДИ, щойно маятник опиняється у верхній зоні (|θ| < 25°),
    незалежно від MPC (після ручного розгойдування, підкидання мишкою тощо).
  - MPC подає силу виключно на візок без жодної телепортації стану чи кутів.
  - Маятник рухається суто за законами нелінійної фізики RK4 (пасивні шарніри).
"""

from enum import Enum
from typing import Dict, Any, Optional
import numpy as np

try:
    from .config import SystemConfig, DEFAULT_CONFIG
    from .dynamics import DoublePendulumDynamics, wrap_to_pi
    from .lqr_controller import LQRController
    from .down_lqr_controller import DownLQRController
    from .mpc_controller import MPCTrajectoryController
except ImportError:
    from config import SystemConfig, DEFAULT_CONFIG
    from dynamics import DoublePendulumDynamics, wrap_to_pi
    from lqr_controller import LQRController
    from down_lqr_controller import DownLQRController
    from mpc_controller import MPCTrajectoryController


class ControlMode(Enum):
    FALLEN = "Вільний рух / Очікування"
    LQR_DOWN_ZERO = "Занулення вниз (Down-LQR)"
    MPC_SWINGUP = "Розгін візка (MPC)"
    LQR_BALANCE = "Балансування (LQR)"
    MANUAL_OFF = "Мотор вимкнено"


class SimulationManager:
    """
    Головний координатор гібридного керування подвійним маятником.
    Реалізує незалежне включення LQR, чисту фізику візка і маятника та телеметрію.
    """

    def __init__(self, config: SystemConfig = DEFAULT_CONFIG):
        self.cfg = config
        self.dynamics = DoublePendulumDynamics(config)
        self.lqr = LQRController(config)
        self.down_lqr = DownLQRController(config)
        self.mpc = MPCTrajectoryController(config)

        # Початковий режим: активне утримання ідеального нульового стану
        self.mode = ControlMode.LQR_DOWN_ZERO
        self.target_cart_x: float = 0.0

        # Зовнішня сила взаємодії (миша, клавіатура)
        self.ext_force: float = 0.0
        self.status_message: str = "✓ Маятник у нульовому стані. Натисніть [MPC Swing-Up] або [Занулити (LQR)]"

    def reset(self, start_down: bool = True) -> None:
        """Скидає симуляцію у початковий стан."""
        self.mpc.stop()
        if start_down:
            self.dynamics.reset(state=np.array([0.0, np.pi, np.pi, 0.0, 0.0, 0.0], dtype=np.float64))
            self.mode = ControlMode.LQR_DOWN_ZERO
            self.status_message = "✓ Маятник у нульовому стані (x=0, θ=π). Натисніть [MPC Swing-Up]"
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
        self.status_message = "Активне занулення (Down-LQR): центрування візка в 0 та гасіння коливань..."

    def trigger_mpc_swingup(self) -> bool:
        """
        Запускає подачу сили розгону візка за маневром MPC.
        КРИТИЧНО: ЖОДНОЇ ТЕЛЕПОРТАЦІЇ! Поточний стан маятника та координата візка
        залишаються рівно такими, якими вони є в момент натискання.
        """
        if not self.mpc.loaded:
            self.status_message = "Помилка: файл траєкторії MPC не знайдено!"
            return False

        self.mpc.start()
        self.mode = ControlMode.MPC_SWINGUP
        self.status_message = "Виконується розгін візка силою мотора MPC..."
        return True

    def toggle_motor(self) -> None:
        """Вмикає або вимикає привід візка."""
        if self.mode == ControlMode.MANUAL_OFF:
            th1_w = abs(wrap_to_pi(self.dynamics.state[1]))
            th2_w = abs(wrap_to_pi(self.dynamics.state[2]))
            catch_rad = np.radians(self.cfg.catch_angle_deg)
            if th1_w < catch_rad and th2_w < catch_rad:
                self.mode = ControlMode.LQR_BALANCE
                self.target_cart_x = self.dynamics.state[0]
                self.status_message = "Мотор увімкнено: LQR підхопив керування"
            else:
                self.mode = ControlMode.FALLEN
                self.status_message = "Мотор увімкнено: вільний рух"
        else:
            self.mpc.stop()
            self.mode = ControlMode.MANUAL_OFF
            self.status_message = "Мотор вимкнено (вільне хитання)"

    def apply_impulse(self, force_magnitude: float) -> None:
        """Додає горизонтальний силовий імпульс на візок (від стрілок)."""
        self.ext_force += force_magnitude

    def set_mouse_drag_target(self, target_x: float) -> None:
        """
        Розраховує реальну пружинно-демпферну силу тяги від курсора миші на візок.
        """
        cart_x = self.dynamics.state[0]
        cart_v = self.dynamics.state[3]
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
        catch_omega = np.radians(self.cfg.catch_max_omega_deg)

        th1_w = abs(wrap_to_pi(self.dynamics.state[1]))
        th2_w = abs(wrap_to_pi(self.dynamics.state[2]))
        w1 = abs(self.dynamics.state[4])
        w2 = abs(self.dynamics.state[5])

        is_upright = bool(
            th1_w < catch_rad and th2_w < catch_rad and
            w1 < catch_omega and w2 < catch_omega
        )

        if is_upright and self.mode != ControlMode.MANUAL_OFF:
            if self.mode != ControlMode.LQR_BALANCE:
                if self.mode == ControlMode.MPC_SWINGUP:
                    self.mpc.stop()
                self.mode = ControlMode.LQR_BALANCE
                self.target_cart_x = self.dynamics.state[0]
                self.status_message = "⚡ LQR підхопив керування! Балансування активовано"

        # =====================================================================
        # 2. ВИЗНАЧЕННЯ СИЛИ МОТОРА ВІЗКА ЗА РЕЖИМОМ КЕРУВАННЯ
        # =====================================================================
        u_control = 0.0

        if self.mode == ControlMode.LQR_BALANCE:
            u_sat, u_raw, is_sat = self.lqr.compute_force(
                self.dynamics.state,
                target_x=self.target_cart_x
            )
            u_control = u_sat

            # Якщо збурення занадто велике і маятник вилетів із зони балансу:
            fall_rad = np.radians(self.cfg.fall_angle_deg)
            if th1_w > fall_rad or th2_w > fall_rad:
                self.mode = ControlMode.FALLEN
                self.status_message = "⚠ Маятник впав! Натисніть [MPC Swing-Up] або розгойдайте візок"

        elif self.mode == ControlMode.LQR_DOWN_ZERO:
            u_sat, u_raw, is_sat = self.down_lqr.compute_force(
                self.dynamics.state,
                target_x=0.0
            )
            u_control = u_sat

            if self.down_lqr.is_zeroed(self.dynamics.state, target_x=0.0):
                self.status_message = "✓ Маятник ідеально занулено (x=0, θ=π)! Можна запускати [MPC Swing-Up]"

        elif self.mode == ControlMode.MPC_SWINGUP:
            self.mpc.playback_time += dt
            t_curr = self.mpc.playback_time

            # Дискретний крок Zero-Order Hold (ZOH) сили MPC
            u_zoh = self.mpc.get_u_zoh(t_curr)

            # Локальна компенсація опору рейки мотором візка (STM32 feedforward loop):
            # u_motor = u_mpc + b_cart * v
            u_control = u_zoh + self.cfg.b_cart * self.dynamics.state[3]

            # Якщо час маневру вичерпався, а маятник не підхопив LQR:
            if t_curr >= self.mpc.duration:
                self.mpc.stop()
                self.mode = ControlMode.FALLEN
                self.status_message = "Маневр MPC завершено"

        elif self.mode in (ControlMode.FALLEN, ControlMode.MANUAL_OFF):
            u_control = 0.0

        # =====================================================================
        # 3. ЧИСТА НЕЕЛІНІЙНА ДИНАМІКА RK4
        # Мотор діє ТІЛЬКИ на візок горизонтальною силою u_control.
        # Жодного впливу на шарніри чи штучного руху маятника немає!
        # Якщо затиснути візок мишкою, сила пружини мишки зупиняє візок,
        # і маятник рухається суто від гравітації та інерції.
        # =====================================================================
        self.dynamics.step(u_control, ext_force=self.ext_force, dt=dt)

        # Згасання разового імпульсу клавіатури
        if abs(self.ext_force) > 0.0 and self.mode != ControlMode.MPC_SWINGUP:
            self.ext_force *= 0.85
            if abs(self.ext_force) < 0.1:
                self.ext_force = 0.0

    def get_info(self) -> Dict[str, Any]:
        """Повертає телеметрію для інтерфейсу."""
        telem = self.dynamics.get_telemetry()
        telem["mode"] = self.mode.value
        telem["status_message"] = self.status_message
        telem["mpc_progress"] = self.mpc.get_progress()
        telem["is_mpc_active"] = bool(self.mode == ControlMode.MPC_SWINGUP)
        telem["is_zeroed"] = bool(self.down_lqr.is_zeroed(self.dynamics.state, target_x=0.0))
        telem["is_down_lqr_active"] = bool(self.mode == ControlMode.LQR_DOWN_ZERO)
        return telem
