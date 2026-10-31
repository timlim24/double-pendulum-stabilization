"""
mpc_controller.py
=================
Контролер відтворення попередньо розрахованої оптимальної траєкторії MPC (Swing-Up Feedforward).
Завантажує файл 'mpc_trajectory.npz', транслює послідовність керуючих зусиль u_mpc(t)
та опорні точки траєкторії [x_ref, v_ref, th_ref, w_ref].
"""

from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

try:
    from .config import DEFAULT_CONFIG, SinglePendulumConfig
    from .dynamics import wrap_to_pi
except ImportError:
    from config import DEFAULT_CONFIG, SinglePendulumConfig
    from dynamics import wrap_to_pi


class MPCTrajectoryController:
    """
    Клас програвача оптимальної траєкторії MPC для одинарного перевернутого маятника.
    """

    def __init__(
        self,
        cfg: SinglePendulumConfig = DEFAULT_CONFIG,
        trajectory_file: Optional[str] = None,
    ):
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
            self.time_arr = data["time"]
            self.u_arr = data["u"]
            self.pos_arr = data["pos"]
            self.th_arr = data["theta"]
            self.v_arr = data["v"]
            self.w_arr = data["omega"]
            self.duration = float(data["total_time"])
            self.dt_step = (
                float(data["dt"])
                if "dt" in data
                else (self.time_arr[1] - self.time_arr[0])
            )
            self.loaded = True
            print(
                f"✓ Траєкторію MPC (одинарний маятник) завантажено: {self.duration:.2f} с ({len(self.time_arr)} точок)."
            )
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

    def get_reference_state(self, t: float) -> Tuple[float, float, float, float]:
        """
        Повертає інтерпольовані опорні значення [x_ref, th_ref, v_ref, w_ref].
        """
        if not self.loaded or len(self.time_arr) == 0:
            return 0.0, 0.0, 0.0, 0.0

        idx = int(t / self.dt_step)
        if idx >= len(self.time_arr) - 1:
            return (
                float(self.pos_arr[-1]),
                float(self.th_arr[-1]),
                float(self.v_arr[-1]),
                float(self.w_arr[-1]),
            )

        frac = (t - idx * self.dt_step) / self.dt_step
        frac = max(0.0, min(1.0, frac))

        x_ref = (1.0 - frac) * self.pos_arr[idx] + frac * self.pos_arr[idx + 1]
        th_ref = (1.0 - frac) * self.th_arr[idx] + frac * self.th_arr[idx + 1]
        v_ref = (1.0 - frac) * self.v_arr[idx] + frac * self.v_arr[idx + 1]
        w_ref = (1.0 - frac) * self.w_arr[idx] + frac * self.w_arr[idx + 1]
        return float(x_ref), float(th_ref), float(v_ref), float(w_ref)

    def step(self, dt: float, current_state: np.ndarray) -> Tuple[float, bool, bool]:
        """
        Крок відтворення траєкторії на інтервал часу dt.
        Повертає кортеж: (u_force, should_handover_to_lqr, is_finished)
        """
        if not self.is_active or not self.loaded:
            return 0.0, False, True

        self.playback_time += dt
        u_ff = self.get_u_zoh(self.playback_time)
        finished = bool(self.playback_time >= self.duration)

        # Перевірка умови входу в зону захоплення LQR
        x_val, th_val, v_val, w_val = current_state
        th_err = wrap_to_pi(th_val)
        catch_rad = np.radians(self.cfg.catch_angle_deg)

        # Якщо маятник наблизився до вертикалі і кутова швидкість помірна
        in_catch_zone = bool(
            abs(th_err) < catch_rad and abs(w_val) < self.cfg.catch_max_omega
        )
        should_handover = in_catch_zone or (finished and abs(th_err) < catch_rad * 1.5)

        if should_handover or finished:
            self.stop()

        # Додатковий зворотний зв'язок відхилення візка від траєкторії (position tracking correction)
        x_ref, _, v_ref, _ = self.get_reference_state(self.playback_time)
        u_track = 15.0 * (x_ref - x_val) + 2.0 * (v_ref - v_val)
        u_total = u_ff + u_track

        return float(u_total), should_handover, finished
