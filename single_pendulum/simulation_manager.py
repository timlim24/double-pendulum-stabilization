"""
simulation_manager.py
=====================
Цифровий двійник стенду: ТЕ САМЕ C-ядро керування, що працює на ESP32
(common/controller.c), + модель стенду (common/host/plant_sim.c) з
ідентифікованими параметрами (common/pendulum_params.json).

Різниця з реальністю зводиться до точності параметрів моделі (їх дає sys_id.py),
а не до різної логіки: стани, пороги, фільтри, LQR, траєкторія підйому,
обмеження швидкості/прискорення — однакові.
"""

import collections
import copy

try:
    from . import controller_lib as cl
    from .params import PendulumParams, load_params
except ImportError:
    import controller_lib as cl
    from params import PendulumParams, load_params


class SimulationManager:
    def __init__(self, params: PendulumParams = None, x0: float = 0.08):
        self.params = copy.deepcopy(params or load_params())
        self.x0 = x0
        self.events = collections.deque(maxlen=8)
        self._time_acc = 0.0
        self._pending_cmd = 0
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Перезапуск "стенду": візок не в центрі, маятник висить, нове ввімкнення."""
        gains = self.bench.ctrl.get_gains() if hasattr(self, "bench") else None
        self.bench = cl.Bench(self.params, x0=self.x0, enc_off=1000)
        if gains:
            self.bench.ctrl.set_gains(gains)
        self.events.clear()
        self.events.append("Старт: хомінг -> центр -> гасіння коливань")
        self._seen = 0

    def command(self, cmd: int) -> None:
        self._pending_cmd |= cmd

    def kick(self, d_omega: float) -> None:
        """Поштовх маятника рукою (миттєва зміна кутової швидкості)."""
        self.bench.plant.kick(d_omega)
        self.events.append(f"Поштовх маятника Δω = {d_omega:+.1f} рад/с")

    def set_model_scale(self, omega0_scale: float = 1.0) -> None:
        """Розбіжність моделі: "справжній" маятник відрізняється від ідентифікованого."""
        p = copy.deepcopy(self.params)
        p.omega0_sq *= omega0_scale
        self.bench.plant.set_params(p)

    def get_gains(self):
        return self.bench.ctrl.get_gains()

    def set_gains(self, g) -> None:
        self.bench.ctrl.set_gains(g)

    # ------------------------------------------------------------------
    def update(self, dt_real: float) -> None:
        """Прокручує стільки тактів 500 Гц, скільки минуло реального часу."""
        self._time_acc += dt_real
        n = int(self._time_acc / cl.CTRL_DT)
        self._time_acc -= n * cl.CTRL_DT
        for _ in range(min(n, 50)):
            self.bench.step(self._pending_cmd)
            self._pending_cmd = 0
        log = self.bench.log_events
        for t, _, text in log[self._seen:]:
            self.events.append(f"{t:6.2f} с  {text}")
        self._seen = len(log)

    # ------------------------------------------------------------------
    @property
    def telemetry(self) -> dict:
        return self.bench.ctrl.telemetry()

    @property
    def plant(self) -> dict:
        return self.bench.plant.get()

    @property
    def state_name(self) -> str:
        return cl.state_name(self.telemetry["state"])
