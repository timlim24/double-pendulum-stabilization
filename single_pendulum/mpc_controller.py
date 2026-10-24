"""
mpc_controller.py
=================
Контролер відтворення попередньо розрахованої оптимальної траєкторії MPC (Swing-Up Feedforward).
Завантажує файл 'mpc_trajectory.npz', транслює послідовність керуючих зусиль u_mpc(t)
та передає керування LQR-регулятору при входженні маятника у зону захоплення (Catch Zone).
"""

from pathlib import Path
from typing import Tuple, Optional
import numpy as np

try:
    from .config import SinglePendulumConfig, DEFAULT_CONFIG
except ImportError:
    from config import SinglePendulumConfig, DEFAULT_CONFIG


class SinglePendulumMPCController:
    """
    Клас програвача траєкторії MPC для одинарного перевернутого маятника.
    """

    def __init__(self, cfg: SinglePendulumConfig = DEFAULT_CONFIG, trajectory_file: Optional[str] = None):
        self.cfg = cfg
        if trajectory_file is None:
            self.traj_path = Path(__file__).resolve().parent / "mpc_trajectory.npz"
        else:
            self.traj_path = Path(trajectory_file)

        self.loaded = False
        self.time_arr = np.array([])
        self.u_arr = np.array([])
        self.pos_arr = np.array([])
        self.th_arr = np.array([])
        self.v_arr = np.array([])
        self.w_arr = np.array([])
        self.duration: float = 0.0
        self.dt_step: float = 0.02

        self.is_active: bool = False
        self.playback_time: float = 0.0

        self.load_trajectory()

    def load_trajectory(self) -> bool:
        """Завантажує траєкторію з .npz файлу."""
        if not self.traj_path.is_file():
            print(f"[Попередження] Файл траєкторії MPC не знайдено: {self.traj_path}")
            self.loaded = False
            return False

        try:
            data = np.load(str(self.traj_path))
            self.time_arr = data['time']
            self.u_arr = data['u']
            self.pos_arr = data['pos']
            self.th_arr = data['theta']
            self.v_arr = data['v']
            self.w_arr = data['omega']
            self.duration = float(data['total_time'])
            self.dt_step = float(data['dt']) if 'dt' in data else (self.time_arr[1] - self.time_arr[0])
            self.loaded = True
            print(f"✓ Траєкторію MPC (одинарний маятник) завантажено: {self.duration:.2f} с ({len(self.time_arr)} точок).")
            return True
        except Exception as e:
            print(f"[Помилка] Не вдалося зчитати траєкторію MPC: {e}")
            self.loaded = False
            return False

    def start(self) -> None:
        """Активує виконання маневру MPC спочатку."""
        self.playback_time = 0.0
        self.is_active = True

    def stop(self) -> None:
        """Зупиняє виконання маневру."""
        self.is_active = False

    def get_u_zoh(self, t: float) -> float:
        """
        Повертає керуючу силу за принципом Zero-Order Hold (ZOH).
        """
        if not self.loaded or len(self.u_arr) == 0:
            return 0.0
        idx = min(len(self.u_arr) - 1, max(0, int(t / self.dt_step)))
        return float(self.u_arr[idx])

    def step(self, dt: float, current_state: np.ndarray) -> Tuple[float, bool, bool]:
        """
        Крок відтворення на інтервал часу dt.
        Повертає кортеж: (u_force, should_handover_to_lqr, is_finished)
        """
        if not self.is_active or not self.loaded:
            return 0.0, False, True

        self.playback_time += dt
        u_ff = self.get_u_zoh(self.playback_time)
        finished = bool(self.playback_time >= self.duration)

        # Перевірка умови входу в зону захоплення LQR
        x_val, th_val, v_val, w_val = current_state
        th_err = (th_val + np.pi) % (2.0 * np.pi) - np.pi
        catch_rad = np.radians(self.cfg.catch_angle_deg)

        # Якщо маятник наблизився до вертикалі і кутова швидкість помірна
        in_catch_zone = bool(abs(th_err) < catch_rad and abs(w_val) < self.cfg.catch_max_omega)
        should_handover = in_catch_zone or (finished and abs(th_err) < catch_rad * 1.5)

        if should_handover or finished:
            self.is_active = False

        return u_ff, should_handover, finished

    def get_progress(self) -> float:
        """Повертає прогрес виконання від 0.0 до 1.0."""
        if not self.loaded or self.duration <= 0.0:
            return 0.0
        return min(1.0, max(0.0, self.playback_time / self.duration))
