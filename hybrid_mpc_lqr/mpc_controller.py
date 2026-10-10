"""
mpc_controller.py
=================
Клас програвача попередньо розрахованої траєкторії MPC (Swing-Up Feedforward).
Завантажує файл 'mpc_trajectory.npz', забезпечує неперервну інтерполяцію
керуючої сили u(t) та опорного стану за часом і перевіряє критерії готовності
до передачі керування LQR-регулятору.
"""

from pathlib import Path
from typing import Tuple, Dict, Any, Optional
import numpy as np
try:
    from .config import SystemConfig, DEFAULT_CONFIG
    from .dynamics import wrap_to_pi
except ImportError:
    from config import SystemConfig, DEFAULT_CONFIG
    from dynamics import wrap_to_pi


class MPCTrajectoryController:
    """
    Контролер відтворення попередньо розрахованого маневру підйому MPC.
    Працює як Feedforward генератор сили u_mpc(t) з плавною часовою інтерполяцією.
    """

    def __init__(self, config: SystemConfig = DEFAULT_CONFIG, trajectory_file: Optional[str] = None):
        self.cfg = config
        if trajectory_file is None:
            self.traj_path = Path(__file__).resolve().parent / "mpc_trajectory.npz"
        else:
            self.traj_path = Path(trajectory_file)

        self.loaded = False
        self.time_arr = np.array([])
        self.u_arr = np.array([])
        self.pos_arr = np.array([])
        self.th1_arr = np.array([])
        self.th2_arr = np.array([])
        self.v_arr = np.array([])
        self.w1_arr = np.array([])
        self.w2_arr = np.array([])
        self.duration: float = 0.0

        # Стан відтворення
        self.is_active: bool = False
        self.playback_time: float = 0.0

        self.load_trajectory()

    def load_trajectory(self) -> bool:
        """Завантажує траєкторію з .npz файлу."""
        if not self.traj_path.is_file():
            print(f"[Увага] Файл траєкторії не знайдено: {self.traj_path}")
            self.loaded = False
            return False

        try:
            data = np.load(str(self.traj_path))
            self.time_arr = data['time']
            self.u_arr = data['u']
            self.pos_arr = data['pos']
            self.th1_arr = data['theta1']
            self.th2_arr = data['theta2']
            self.v_arr = data['v']
            self.w1_arr = data['omega1']
            self.w2_arr = data['omega2']
            self.duration = float(self.time_arr[-1])
            self.loaded = True
            print(f"✓ Траєкторію MPC успішно завантажено! Тривалість: {self.duration:.2f} с ({len(self.time_arr)} точок)")
            return True
        except Exception as e:
            print(f"[Помилка] Не вдалося зчитати траєкторію MPC: {e}")
            self.loaded = False
            return False

    def start(self) -> None:
        """Активує виконання маневру MPC з початку."""
        self.playback_time = 0.0
        self.is_active = True

    def stop(self) -> None:
        """Зупиняє виконання маневру."""
        self.is_active = False

    def get_u_zoh(self, t: float) -> float:
        """
        Повертає силу керування u(t) за принципом Zero-Order Hold (ZOH).
        У дискретному MPC керуючий імпульс є кусково-постійним на кожному інтервалі dt.
        """
        if not self.loaded or len(self.u_arr) == 0:
            return 0.0
        dt_step = float(self.time_arr[1] - self.time_arr[0]) if len(self.time_arr) > 1 else 0.04
        idx = min(len(self.u_arr) - 1, max(0, int(t / dt_step)))
        return float(self.u_arr[idx])

    def step(self, dt: float, current_state: np.ndarray) -> Tuple[float, bool, bool]:
        """
        Робить один крок за часом dt.
        
        Повертає кортеж:
          (u_force, should_handover_to_lqr, trajectory_finished)
        """
        if not self.is_active or not self.loaded:
            return 0.0, False, True

        self.playback_time += dt

        # Сила керування u(t) за дискретним кроком ZOH
        u_ff = self.get_u_zoh(self.playback_time)

        # Перевірка умови закінчення за часом
        finished = bool(self.playback_time >= self.duration)

        # Перевірка умови входження в зону стійкості (Handover)
        catch_angle_rad = np.radians(self.cfg.catch_angle_deg)
        catch_omega_rad = np.radians(self.cfg.catch_max_omega_deg)

        th1_w = abs(wrap_to_pi(current_state[1]))
        th2_w = abs(wrap_to_pi(current_state[2]))
        w1 = abs(current_state[4])
        w2 = abs(current_state[5])

        # Умова передачі керування LQR:
        # маятник підійшов близько до вершини (обидва кути в межах зони захоплення)
        is_in_catch_zone = bool(
            th1_w < catch_angle_rad and
            th2_w < catch_angle_rad and
            w1 < catch_omega_rad and
            w2 < catch_omega_rad
        )

        should_handover = is_in_catch_zone or (finished and th1_w < catch_angle_rad * 1.5)

        if should_handover or finished:
            self.is_active = False

        return u_ff, should_handover, finished

    def get_progress(self) -> float:
        """Повертає прогрес виконання від 0.0 до 1.0."""
        if not self.loaded or self.duration <= 0.0:
            return 0.0
        return min(1.0, max(0.0, self.playback_time / self.duration))

    def get_reference_state(self, t: float) -> np.ndarray:
        """Повертає інтерпольований опорний стан MPC у момент часу t."""
        if not self.loaded:
            return np.zeros(6, dtype=np.float64)

        t_clamped = min(self.duration, max(0.0, t))
        return np.array([
            float(np.interp(t_clamped, self.time_arr, self.pos_arr)),
            float(np.interp(t_clamped, self.time_arr, self.th1_arr)),
            float(np.interp(t_clamped, self.time_arr, self.th2_arr)),
            float(np.interp(t_clamped, self.time_arr, self.v_arr)),
            float(np.interp(t_clamped, self.time_arr, self.w1_arr)),
            float(np.interp(t_clamped, self.time_arr, self.w2_arr))
        ], dtype=np.float64)
