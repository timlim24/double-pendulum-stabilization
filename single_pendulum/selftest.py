#!/usr/bin/env python3
"""
selftest.py
===========
Автоматична перевірка ПРОШИВКИ (того самого C-ядра) у симуляції стенду.

Сценарій: старт не в центрі -> хомінг -> занулення -> підйом -> баланс ->
поштовхи -> падіння -> гасіння -> збитий нуль енкодера (60°) -> SysID.

Перевіряється і на номінальних параметрах, і на "спотворених" (модель стенду
відрізняється від тієї, під яку пораховані коефіцієнти) — запас стійкості.

    python single_pendulum/selftest.py            # усі варіанти
    python single_pendulum/selftest.py --quick    # тільки номінальний
"""

import argparse
import copy
import sys

import numpy as np

try:
    from . import controller_lib as cl
    from .params import load_params
except ImportError:
    import controller_lib as cl
    from params import load_params


class Fail(Exception):
    pass


def run_until(bench, cond, timeout, cmd_first=0, label=""):
    cmd = cmd_first
    t_end = bench.t + timeout
    while bench.t < t_end:
        bench.step(cmd)
        cmd = 0
        if cond(bench):
            return bench.t
    raise Fail(f"таймаут: {label} (стан={cl.state_name(bench.ctrl.telemetry()['state'])})")


def state_is(*states):
    return lambda b: b.ctrl.telemetry()["state"] in states


def scenario(p_true, verbose=False, sysid=True):
    """Повертає словник метрик або кидає Fail."""
    m = {}
    b = cl.Bench(p_true, x0=0.10, phi0=0.0, enc_off=12345)
    b.plant.set_backlash(getattr(p_true, "backlash_m", 0.0))

    # 1. Хомінг та заспокоєння
    run_until(b, state_is(cl.ST_WAITING), 40.0, label="хомінг -> очікування")
    tel = b.ctrl.telemetry()
    m["x_after_home_mm"] = 1000 * (b.plant.get()["x"] - 0.0)
    if abs(b.plant.get()["x"]) > 0.005:
        raise Fail(f"центр після хомінгу хибний: x_true={b.plant.get()['x']:.4f}")

    # 2. Підйом
    t0 = b.t
    run_until(b, state_is(cl.ST_BALANCING, cl.ST_STOPPING), 9.0, cmd_first=cl.CMD_SWINGUP,
              label="підйом")
    if b.ctrl.telemetry()["state"] != cl.ST_BALANCING:
        raise Fail("підйом не вдався: " + b.log_events[-1][2])
    m["swingup_s"] = b.t - t0
    xs, ths = [], []
    for _ in range(int(6.0 / cl.CTRL_DT)):
        b.step()
        tel = b.ctrl.telemetry()
        xs.append(b.plant.get()["x"])
        ths.append(tel["theta"])
        if tel["state"] != cl.ST_BALANCING:
            raise Fail("впав під час балансу після підйому")
    m["bal_x_rms_mm"] = 1000 * float(np.sqrt(np.mean(np.square(xs[-1500:]))))
    m["bal_th_rms_deg"] = float(np.degrees(np.sqrt(np.mean(np.square(ths[-1500:])))))

    # 3. Поштовх маятника під час балансу (0.8 рад/с ≈ 0.28 м/с на кінці стрижня)
    b.plant.kick(0.8)
    mx = 0.0
    for _ in range(int(4.0 / cl.CTRL_DT)):
        b.step()
        mx = max(mx, abs(b.plant.get()["x"]))
        if b.ctrl.telemetry()["state"] != cl.ST_BALANCING:
            raise Fail("не втримав поштовх 0.8 рад/с")
    m["max_x_after_kick_mm"] = 1000 * mx

    # 4. Скасування балансу [x] -> маятник падає -> гасіння внизу
    run_until(b, state_is(cl.ST_STOPPING), 0.1, cmd_first=cl.CMD_ABORT, label="скасування")
    run_until(b, state_is(cl.ST_ZEROING), 3.0, label="падіння")
    t0 = b.t
    run_until(b, state_is(cl.ST_WAITING), 60.0, label="гасіння після падіння")
    m["zeroing_after_fall_s"] = b.t - t0

    # 5. Збитий нуль енкодера на 60° (як після маневру на стенді)
    b.plant.reset(b.plant.get()["x"], b.plant.get()["phi"], 0.0, enc_off=12345 + 400)
    b.plant.set_backlash(getattr(p_true, "backlash_m", 0.0))
    b.plant.set_measured_x(b.ctrl.telemetry()["x"])
    t0 = b.t
    run_until(b, lambda bb: any(c == cl.EV_ZERO_CORRECTED for _, c, _ in bb.log_events),
              10.0, label="автокорекція нуля")
    run_until(b, state_is(cl.ST_WAITING), 20.0, label="очікування після корекції нуля")
    m["zero_fix_s"] = b.t - t0
    if abs(b.ctrl.telemetry()["phi"]) > np.radians(1.0):
        raise Fail("нуль скориговано неточно")

    # 6. Перевірка: стопери ніколи не торкались, X не "з'їхав"
    stoppers = [e for e in b.log_events if e[1] == cl.EV_STOPPER]
    if stoppers:
        raise Fail(f"торкання стопера: {stoppers[0][2]}")
    m["x_estimate_err_mm"] = 1000 * abs(b.ctrl.telemetry()["x"] - b.plant.get()["x"])
    # Лічильник кроків бачить вал; візок відстає максимум на величину люфту
    if m["x_estimate_err_mm"] > 0.5 + 1000 * getattr(p_true, "backlash_m", 0.0):
        raise Fail(f"оцінка X розійшлась з реальністю на {m['x_estimate_err_mm']:.2f} мм")

    # 7. SysID (3 тріали, буфер "вивантажується" миттєво)
    if sysid:
        run_until(b, lambda bb: any("усі тріали" in e[2] for e in bb.log_events), 150.0,
                  cmd_first=cl.CMD_SYSID, label="SysID")
        m["sysid_samples"] = {k: len(v) for k, v in b.sysid.items()}
        m["sysid_max_x_mm"] = 1000 * max(abs(s[1]) for v in b.sysid.values() for s in v)

    if verbose:
        for t, _, text in b.log_events:
            print(f"   {t:7.2f} с  {text}")
    return m, b


def variants(p):
    out = [("номінал", p)]
    for name, kw in [
        ("ω0² −5%", {"omega0_sq": p.omega0_sq * 0.95}),
        ("ω0² +5%", {"omega0_sq": p.omega0_sq * 1.05}),
        ("κ 0.85 (візок недотягує)", {"kappa": 0.85}),
        ("κ 1.15", {"kappa": 1.15}),
        ("τ 12 мс (люфт/пас)", {"tau_act": 0.012}),
        ("сухе тертя 0.4", {"c_coul": 0.4}),
        ("в'язке тертя ×3", {"d_visc": p.d_visc * 3}),
        ("люфт паса 1 мм", {"backlash_m": 0.001}),
    ]:
        q = copy.deepcopy(p)
        for k, v in kw.items():
            setattr(q, k, v)
        out.append((name, q))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    p = load_params()
    vs = variants(p)[:1] if args.quick else variants(p)
    ok = True
    for i, (name, q) in enumerate(vs):
        try:
            m, _ = scenario(q, verbose=args.verbose and i == 0, sysid=(i == 0))
            print(f"✓ {name:28s} " + ", ".join(
                f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in m.items()))
        except Fail as e:
            ok = False
            print(f"✗ {name:28s} {e}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
