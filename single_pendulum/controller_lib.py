"""
controller_lib.py
=================
Python-обгортка (ctypes) над ТИМ САМИМ C-кодом, що працює на ESP32:
  common/controller.c        — ядро керування (стани, оцінювач, LQR, підйом, SysID)
  common/host/plant_sim.c    — модель стенду (кроковий візок, маятник, енкодер, стопери)

Бібліотека збирається автоматично (gcc) при першому імпорті або якщо
змінилися вихідні файли/заголовки.
"""

import ctypes as C
import subprocess
from pathlib import Path

import numpy as np

try:
    from .hwconfig import COMMON_DIR, EVENTS, HW, STATES
except ImportError:
    from hwconfig import COMMON_DIR, EVENTS, HW, STATES

BUILD_DIR = Path(__file__).resolve().parent / "_build"
LIB_PATH = BUILD_DIR / "libcartpole.so"
SOURCES = [
    COMMON_DIR / "controller.c",
    COMMON_DIR / "host" / "controller_host_api.c",
    COMMON_DIR / "host" / "plant_sim.c",
]
HEADERS = [
    COMMON_DIR / "controller.h",
    COMMON_DIR / "cart_pole_config.h",
    COMMON_DIR / "pendulum_params.h",
    COMMON_DIR / "swingup_trajectory.h",
]

# Команди (бітова маска з controller.h)
CMD_SWINGUP = 1 << 0
CMD_SET_CENTER = 1 << 1
CMD_HOME = 1 << 2
CMD_SYSID = 1 << 3
CMD_ABORT = 1 << 4

# Стани та події читаються з controller.h (без ручного дублювання номерів)
globals().update({k: v for k, v in STATES.items() if k != "ST_COUNT"})
globals().update({k: v for k, v in EVENTS.items() if k != "EV_COUNT"})


def build(force: bool = False) -> Path:
    BUILD_DIR.mkdir(exist_ok=True)
    newest = max(p.stat().st_mtime for p in SOURCES + HEADERS)
    if force or not LIB_PATH.exists() or LIB_PATH.stat().st_mtime < newest:
        cmd = [
            "gcc", "-O2", "-shared", "-fPIC", "-std=c11", "-Wall", "-Wextra",
            "-Wno-unused-parameter", "-D_GNU_SOURCE",
            f"-I{COMMON_DIR}", "-o", str(LIB_PATH), *map(str, SOURCES), "-lm",
        ]
        subprocess.run(cmd, check=True)
    return LIB_PATH


def _load():
    lib = C.CDLL(str(build()))
    fp = C.POINTER(C.c_float)
    dp = C.POINTER(C.c_double)
    sig = {
        "api_create": (C.c_void_p, [C.c_int32]),
        "api_destroy": (None, [C.c_void_p]),
        "api_step": (None, [C.c_void_p, C.c_int32, C.c_float, C.c_int, C.c_int, C.c_uint32, C.c_int]),
        "api_output": (None, [C.c_void_p, fp]),
        "api_telemetry": (None, [C.c_void_p, fp]),
        "api_n_events": (C.c_int, [C.c_void_p]),
        "api_event": (C.c_int, [C.c_void_p, C.c_int, C.POINTER(C.c_int), C.c_char_p, C.c_int]),
        "api_sysid": (None, [C.c_void_p, fp]),
        "api_get_gains": (None, [C.c_void_p, fp]),
        "api_set_gains": (None, [C.c_void_p, fp]),
        "api_state_name": (C.c_char_p, [C.c_int]),
        "api_sysid_profile": (None, [C.c_int, C.c_float, fp, fp]),
        "api_sysid_duration": (C.c_float, [C.c_int]),
        "api_const": (C.c_float, [C.c_char_p]),
        "plant_create": (C.c_void_p, [C.c_double] * 5),
        "plant_destroy": (None, [C.c_void_p]),
        "plant_set_params": (None, [C.c_void_p] + [C.c_double] * 5),
        "plant_reset": (None, [C.c_void_p, C.c_double, C.c_double, C.c_double, C.c_int32]),
        "plant_kick": (None, [C.c_void_p, C.c_double]),
        "plant_set_backlash": (None, [C.c_void_p, C.c_double]),
        "plant_set_measured_x": (None, [C.c_void_p, C.c_double]),
        "plant_step": (None, [C.c_void_p, C.c_double, C.c_int, C.c_double]),
        "plant_get": (None, [C.c_void_p, dp]),
        "plant_simulate": (None, [dp, C.c_double, C.c_double, C.c_int, C.c_double, dp, dp]),
    }
    for name, (res, args) in sig.items():
        fn = getattr(lib, name)
        fn.restype = res
        fn.argtypes = args
    return lib


LIB = _load()
CTRL_DT = float(LIB.api_const(b"CTRL_DT"))
DEBOUNCE_SAMPLES = int(HW["STOPPER_DEBOUNCE_SAMPLES"])


def state_name(s: int) -> str:
    return LIB.api_state_name(int(s)).decode("utf-8")


def _fbuf(n):
    return (C.c_float * n)()


class Controller:
    """Ядро керування стенду (точна копія логіки прошивки)."""

    def __init__(self, enc_ticks: int = 0):
        self.h = LIB.api_create(int(enc_ticks))
        self._o4, self._t12, self._s7, self._g5 = _fbuf(4), _fbuf(12), _fbuf(7), _fbuf(5)
        self._ev = C.create_string_buffer(256)

    def __del__(self):
        if getattr(self, "h", None):
            LIB.api_destroy(self.h)
            self.h = None

    def step(self, ticks, cart_x, stop_l, stop_r, cmd=0, buffer_free=True):
        LIB.api_step(self.h, int(ticks), float(cart_x), int(stop_l), int(stop_r), int(cmd), int(buffer_free))
        LIB.api_output(self.h, self._o4)
        o = self._o4
        return o[0], bool(o[1]), bool(o[2]), o[3]

    def events(self):
        res = []
        code = C.c_int()
        for i in range(LIB.api_n_events(self.h)):
            LIB.api_event(self.h, i, C.byref(code), self._ev, 256)
            res.append((code.value, self._ev.value.decode("utf-8", errors="replace")))
        return res

    def telemetry(self) -> dict:
        LIB.api_telemetry(self.h, self._t12)
        t = list(self._t12)
        return {
            "state": int(t[0]), "t": t[1], "x": t[2], "v": t[3], "a": t[4],
            "theta": t[5], "phi": t[6], "omega": t[7], "homed": bool(t[8]),
            "swing_t": t[9], "sysid_trial": int(t[10]), "enc_zero": int(t[11]),
        }

    def sysid_sample(self):
        LIB.api_sysid(self.h, self._s7)
        s = list(self._s7)
        return {
            "sample": bool(s[0]), "trial": int(s[1]), "index": int(s[2]),
            "x": s[3], "v": s[4], "ticks": int(s[5]), "trial_done": bool(s[6]),
        }

    def get_gains(self):
        LIB.api_get_gains(self.h, self._g5)
        return list(self._g5)

    def set_gains(self, g):
        buf = (C.c_float * 5)(*[float(v) for v in g])
        LIB.api_set_gains(self.h, buf)


class Plant:
    """Модель стенду: кроковий візок + маятник + енкодер + стопери."""

    def __init__(self, params):
        self.h = LIB.plant_create(*self._pvec(params))
        self._o8 = (C.c_double * 8)()

    @staticmethod
    def _pvec(p):
        return [float(p.omega0_sq), float(p.gamma), float(p.d_visc), float(p.c_coul), float(p.tau_act)]

    def __del__(self):
        if getattr(self, "h", None):
            LIB.plant_destroy(self.h)
            self.h = None

    def set_params(self, params):
        LIB.plant_set_params(self.h, *self._pvec(params))

    def reset(self, x=0.0, phi=0.0, omega=0.0, enc_off=0):
        LIB.plant_reset(self.h, x, phi, omega, int(enc_off))

    def kick(self, d_omega):
        LIB.plant_kick(self.h, float(d_omega))

    def set_backlash(self, meters):
        """Люфт пас/шків (м): вал крутиться, а візок стоїть, доки зазор не вибрано."""
        LIB.plant_set_backlash(self.h, float(meters))

    def set_measured_x(self, value):
        LIB.plant_set_measured_x(self.h, float(value))

    def step(self, v_cmd, hard_stop, dt):
        LIB.plant_step(self.h, float(v_cmd), int(hard_stop), float(dt))

    def get(self) -> dict:
        LIB.plant_get(self.h, self._o8)
        o = list(self._o8)
        return {
            "x": o[0], "v": o[1], "phi": o[2], "omega": o[3], "x_meas": o[4],
            "ticks": int(o[5]), "stop_l": bool(o[6]), "stop_r": bool(o[7]),
        }


def simulate_open_loop(pvec, phi0, w0, dt, v_in):
    """Відкритий контур (для SysID). pvec = [ω0², γ, d, c, τ]. Повертає φ(t)."""
    v = np.ascontiguousarray(v_in, dtype=np.float64)
    out = np.zeros_like(v)
    pv = np.ascontiguousarray(pvec, dtype=np.float64)
    dp = C.POINTER(C.c_double)
    LIB.plant_simulate(pv.ctypes.data_as(dp), float(phi0), float(w0), len(v), float(dt),
                       v.ctypes.data_as(dp), out.ctypes.data_as(dp))
    return out


def sysid_profile(trial, t):
    x, v = C.c_float(), C.c_float()
    LIB.api_sysid_profile(int(trial), float(t), C.byref(x), C.byref(v))
    return x.value, v.value


def sysid_duration(trial):
    return float(LIB.api_sysid_duration(int(trial)))


class Bench:
    """Замкнений контур: контролер + модель стенду, крок 500 Гц (як на ESP32)."""

    def __init__(self, params, x0=0.0, phi0=0.0, omega0=0.0, enc_off=0):
        self.plant = Plant(params)
        self.plant.reset(x0, phi0, omega0, enc_off)
        st = self.plant.get()
        self.ctrl = Controller(st["ticks"])
        self.v_cmd = 0.0
        self.hard = False
        self.t = 0.0
        self.log_events = []
        self.sysid = {}
        self._deb = {"l": [False, 0], "r": [False, 0]}

    def _debounce(self, key, raw):
        """Той самий дебаунс кінцевиків, що в esp32_cart_pole/main/stoppers.c."""
        state = self._deb[key]
        if raw != state[0]:
            state[1] += 1
            if state[1] >= DEBOUNCE_SAMPLES:
                state[0], state[1] = raw, 0
        else:
            state[1] = 0
        return state[0]

    def step(self, cmd=0, buffer_free=True):
        self.plant.step(self.v_cmd, self.hard, CTRL_DT)
        st = self.plant.get()
        st["stop_l"] = self._debounce("l", st["stop_l"])
        st["stop_r"] = self._debounce("r", st["stop_r"])
        v, hard, set_x, set_val = self.ctrl.step(
            st["ticks"], st["x_meas"], st["stop_l"], st["stop_r"], cmd, buffer_free
        )
        if set_x:
            self.plant.set_measured_x(set_val)
        self.v_cmd, self.hard = v, hard
        self.t += CTRL_DT
        for code, text in self.ctrl.events():
            self.log_events.append((self.t, code, text))
        s = self.ctrl.sysid_sample()
        if s["sample"]:
            self.sysid.setdefault(s["trial"], []).append((s["index"], s["x"], s["v"], s["ticks"]))
        return st
