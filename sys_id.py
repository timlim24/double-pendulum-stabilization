#!/usr/bin/env python3
"""
sys_id.py
=========
Системна ідентифікація одинарного маятника на кроковому візку.

Мета — щоб симуляція (single_pendulum/) рухалась ТАК САМО, як стенд.
Модель (та сама, що в прошивці, симуляції та генераторі траєкторії):

    φ'' = −ω0² sinφ − κ(ω0²/g) a cosφ − d φ' − c tanh(φ'/ε)
    візок: швидкість відстає від команди з постійною часу τ (пас/ротор)

Невідомі: ω0², κ (ефективне підсилення приводу; 1 = візок точно виконує
команду), d (в'язке тертя), c (сухе тертя), τ (запізнення приводу) +
для кожного тріалу: зсув нуля енкодера та початковий стан.

Як працює:
 1. Стенд (команда 'i') сам виконує 3 гладкі профілі руху візка (резонанс +
    вільне згасання, chirp 0.4–1.8 Гц, ступені ±5 см), пише в RAM на 250 Гц
    (позиція з лічильника кроків, командна швидкість, тіки енкодера) і після
    кожного тріалу вивантажує дані по UART. Жодних пропусків, справжні мітки часу.
 2. Спільний фіт усіх тріалів (scipy least_squares, симуляція на C).
 3. Перевірка: фіт на тріалах 1+2, прогноз тріалу 3 (не брав участі у фіті).
 4. Запис у common/pendulum_params.json, перегенерація pendulum_params.h,
    траєкторії підйому та самоперевірка прошивки в симуляції.

Запуск:
    python sys_id.py                       # записати зі стенду і підібрати
    python sys_id.py --file sysid_data/X.npz   # повторний фіт збереженого запису
    python sys_id.py --simulate            # перевірка самого фітера на синтетиці
    python sys_id.py --flash               # після фіту зібрати і прошити ESP32
"""

import argparse
import glob
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "single_pendulum"))

import controller_lib as cl  # noqa: E402
from hwconfig import HW  # noqa: E402
from params import G, load_params, save_params, write_c_header  # noqa: E402

DATA_DIR = ROOT / "sysid_data"
PLOT_PATH = ROOT / "sysid_result.png"
RPT = HW["RAD_PER_TICK"]
SPM = HW["STEPS_PER_METER"]
MIN_STEP = HW["MIN_STEP_FREQ"]

GLOBAL_NAMES = ["omega0_sq", "kappa", "d_visc", "c_coul", "tau_act"]
LOWER = [10.0, 0.5, 0.0, 0.0, 0.0005]
UPPER = [90.0, 1.5, 2.0, 2.0, 0.06]


# =============================================================================
# 1. Запис зі стенду
# =============================================================================
def record_from_stand(port, baud, timeout=300.0):
    import serial

    print("\033[1;36m=== SysID: запис зі стенду ===\033[0m")
    print("Маятник має висіти спокійно, візок відкалібрований (після хомінгу).")
    print("Руками нічого не чіпати: 3 тріали ~11+11+9 с + заспокоєння + вивантаження.\n")
    ser = serial.Serial(port, baud, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()

    trials, cur, expected = {}, None, 0
    t0 = time.time()
    last_send = 0.0
    started = False
    while time.time() - t0 < timeout:
        if not started and time.time() - last_send > 2.0:
            ser.write(b"i")
            last_send = time.time()
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        if line.startswith("EVT:"):
            code = int(line[4:].split(" ", 1)[0] or 0)
            print("  " + line.split(" ", 1)[-1])
            if code == cl.EV_SYSID_START:
                started = True
            elif code == cl.EV_CMD_REJECTED and not started:
                print("  (стенд ще не готовий — повторю запит)")
            elif code == cl.EV_SYSID_ALL_DONE:
                break
        elif line.startswith("SYSID_BEGIN"):
            _, trial, n, dt = line.split(",")
            cur = int(trial)
            expected = int(n)
            trials[cur] = {"dt": float(dt), "rows": {}}
            print(f"  ← вивантаження тріалу {cur}: {expected} зразків ...")
        elif line.startswith("D,") and cur is not None:
            try:
                _, i, x, v, ticks = line.split(",")
                trials[cur]["rows"][int(i)] = (float(x), float(v), int(ticks))
            except ValueError:
                pass  # пошкоджений рядок — буде пропуском
        elif line.startswith("SYSID_END") and cur is not None:
            got = len(trials[cur]["rows"])
            print(f"    отримано {got}/{expected}" + ("" if got == expected else "  (пропуски будуть інтерпольовані)"))
            trials[cur]["n"] = expected
            cur = None
    ser.close()
    out = {}
    for k, tr in sorted(trials.items()):
        arr = _rows_to_arrays(tr)
        miss = 1.0 - arr["valid"].mean()
        if "n" not in tr:
            print(f"\033[33m⚠ тріал {k}: не отримано SYSID_END — дані неповні\033[0m")
        if miss > 0.02:
            print(f"\033[31m✗ тріал {k}: втрачено {miss * 100:.1f}% зразків — тріал відкинуто\033[0m")
            continue
        if miss > 0:
            print(f"\033[33m⚠ тріал {k}: втрачено {miss * 100:.2f}% зразків (інтерпольовано)\033[0m")
        out[k] = arr
    if len(out) < 2:
        sys.exit(f"Придатних тріалів: {len(out)} — повторіть запис (перевірте кабель/порт).")
    return out


def _rows_to_arrays(tr):
    n = tr.get("n", max(tr["rows"]) + 1)
    x = np.full(n, np.nan)
    v = np.full(n, np.nan)
    ticks = np.full(n, np.nan)
    for i, (xi, vi, ti) in tr["rows"].items():
        if 0 <= i < n:
            x[i], v[i], ticks[i] = xi, vi, ti
    valid = ~np.isnan(ticks)
    idx = np.arange(n)
    for arr in (x, v, ticks):
        arr[~valid] = np.interp(idx[~valid], idx[valid], arr[valid])
    return {"dt": tr["dt"], "x": x, "v": v, "ticks": ticks, "valid": valid}


# =============================================================================
# 1б. Синтетичні дані (перевірка фітера): прошивка + модель стенду
# =============================================================================
def record_synthetic(true_params):
    print("\033[1;36m=== SysID на синтетиці ===\033[0m")
    print("Справжні параметри:", {k: round(getattr(true_params, k), 4) for k in GLOBAL_NAMES})
    b = cl.Bench(true_params, x0=0.05)
    while b.ctrl.telemetry()["state"] != cl.ST_WAITING:
        b.step()
    b.step(cl.CMD_SYSID)
    while not any(c == cl.EV_SYSID_ALL_DONE for _, c, _ in b.log_events):
        b.step()
    dt = cl.CTRL_DT * HW["SYSID_LOG_DECIM"]
    out = {}
    for k, rows in b.sysid.items():
        rows = sorted(rows)
        out[k] = {
            "dt": dt,
            "x": np.array([r[1] for r in rows]),
            "v": np.array([r[2] for r in rows]),
            "ticks": np.array([r[3] for r in rows], dtype=float),
            "valid": np.ones(len(rows), bool),
        }
    return out


# =============================================================================
# 2. Модель та фіт
# =============================================================================
def deadband(v):
    """Генератор кроків не крутить нижче MIN_STEP_FREQ."""
    return np.where(np.abs(v) * SPM < MIN_STEP, 0.0, v)


def simulate_trial(g, local, tr):
    w2, kappa, d, c, tau = g
    off, phi0, w0 = local
    pvec = [w2, kappa * w2 / G, d, c, tau]
    return cl.simulate_open_loop(pvec, phi0, w0, tr["dt"], deadband(tr["v"])) + off


def initial_guess(trials, prior):
    """ω0 з вільного згасання тріалу 1 (перетини нуля), d — з логарифмічного декременту."""
    g = [prior.omega0_sq, 1.0, max(prior.d_visc, 0.02), 0.02, max(prior.tau_act, 0.003)]
    tr = trials.get(1)
    if tr is not None:
        phi = tr["ticks"] * RPT
        t = np.arange(len(phi)) * tr["dt"]
        still = np.abs(tr["v"]) < 1e-4
        seg = np.where(still & (t > 4.0))[0]
        if len(seg) > 200:
            y = phi[seg] - np.mean(phi[seg])
            ts = t[seg]
            zc = ts[1:][np.diff(np.sign(y)) != 0]
            if len(zc) > 4:
                period = 2 * np.mean(np.diff(zc))
                g[0] = (2 * np.pi / period) ** 2
            peaks = np.abs(y)
            n = len(y) // 2
            a1, a2 = np.max(peaks[:n]), np.max(peaks[n:])
            if a1 > 0 and a2 > 0 and a2 < a1:
                g[2] = float(np.clip(np.log(a1 / a2) / ((ts[-1] - ts[0]) / 2), 0.0, 1.0))
    return g


def _local_guess(tr):
    phi = tr["ticks"] * RPT
    off = float(np.median(phi[: max(5, int(0.25 / tr["dt"]))]))
    return [off, 0.0, 0.0]


def fit(trials, keys, prior, verbose=True):
    keys = list(keys)
    g0 = initial_guess(trials, prior)
    x0 = list(g0)
    lo, hi = list(LOWER), list(UPPER)
    for k in keys:
        x0 += _local_guess(trials[k])
        lo += [-0.6, -0.6, -4.0]
        hi += [0.6, 0.6, 4.0]
    x0 = np.clip(x0, np.array(lo) + 1e-9, np.array(hi) - 1e-9)

    def resid(p):
        g = p[:5]
        r = []
        for i, k in enumerate(keys):
            tr = trials[k]
            local = p[5 + 3 * i: 8 + 3 * i]
            sim = simulate_trial(g, local, tr)
            meas = tr["ticks"] * RPT
            r.append(np.where(tr["valid"], sim - meas, 0.0))
        return np.concatenate(r)

    res = least_squares(resid, x0, bounds=(lo, hi), x_scale="jac", max_nfev=400, verbose=0)
    # Невизначеність параметрів з якобіана
    dof = max(1, len(res.fun) - len(res.x))
    s2 = 2 * res.cost / dof
    try:
        cov = np.linalg.inv(res.jac.T @ res.jac) * s2
        std = np.sqrt(np.clip(np.diag(cov), 0, None))
    except np.linalg.LinAlgError:
        std = np.full(len(res.x), np.nan)
    locals_ = {k: res.x[5 + 3 * i: 8 + 3 * i] for i, k in enumerate(keys)}
    return res.x[:5], std[:5], locals_


def score(trials, g, locals_, k):
    """RMS (°) та R² для тріалу k. Якщо k не фітився — підбирає лише його зсув/початок."""
    tr = trials[k]
    meas = tr["ticks"] * RPT
    if k in locals_:
        local = locals_[k]
    else:
        def r(lp):
            return np.where(tr["valid"], simulate_trial(g, lp, tr) - meas, 0.0)
        local = least_squares(r, _local_guess(tr), bounds=([-0.6, -0.6, -4], [0.6, 0.6, 4])).x
    sim = simulate_trial(g, local, tr)
    err = (sim - meas)[tr["valid"]]
    r2 = 1 - np.sum(err ** 2) / np.sum((meas[tr["valid"]] - np.mean(meas[tr["valid"]])) ** 2)
    return float(np.degrees(np.sqrt(np.mean(err ** 2)))), float(r2 * 100), sim, local


# =============================================================================
# 3. Звіт, графік, збереження
# =============================================================================
def plot(trials, g, locals_, scores, val_note, path=PLOT_PATH):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    titles = {1: "Тріал 1: резонанс + вільне згасання", 2: "Тріал 2: chirp 0.4–1.8 Гц", 3: "Тріал 3: ступені ±5 см"}
    fig, axes = plt.subplots(len(trials), 1, figsize=(12, 3.2 * len(trials)), squeeze=False)
    for ax, k in zip(axes[:, 0], sorted(trials)):
        tr = trials[k]
        t = np.arange(len(tr["ticks"])) * tr["dt"]
        rms, r2, sim, _ = scores[k]
        ax.plot(t, np.degrees(tr["ticks"] * RPT), "b-", lw=1.6, label="Стенд")
        ax.plot(t, np.degrees(sim), "r--", lw=1.3, label=f"Симуляція (RMS {rms:.2f}°, R² {r2:.1f}%)")
        ax2 = ax.twinx()
        ax2.plot(t, tr["x"] * 100, color="0.6", lw=0.8)
        ax2.set_ylabel("X візка, см", color="0.5")
        ax.set_ylabel("Кут від низу, °")
        ax.set_title(titles.get(k, f"Тріал {k}") + (val_note if k == 3 else ""), fontsize=10)
        ax.grid(alpha=0.4, ls="--")
        ax.legend(loc="upper right")
    axes[-1, 0].set_xlabel("Час, с")
    plt.tight_layout()
    Path(path).parent.mkdir(exist_ok=True)
    plt.savefig(path, dpi=130)
    print(f"✓ Графік: {path}")


def find_idf_export():
    cands = []
    if os.environ.get("IDF_PATH"):
        cands.append(Path(os.environ["IDF_PATH"]) / "export.sh")
    cands += [Path(p) for p in sorted(glob.glob(os.path.expanduser("~/.espressif/v*/esp-idf/export.sh")))[::-1]]
    cands.append(Path.home() / "esp" / "esp-idf" / "export.sh")
    return next((c for c in cands if c.exists()), None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--file", help="фіт збереженого запису .npz")
    ap.add_argument("--simulate", action="store_true", help="синтетичні дані з моделі (тест фітера)")
    ap.add_argument("--no-save", action="store_true", help="не змінювати параметри")
    ap.add_argument("--force", action="store_true", help="зберегти навіть при поганій якості")
    ap.add_argument("--no-traj", action="store_true", help="не перераховувати траєкторію підйому")
    ap.add_argument("--flash", action="store_true", help="зібрати та прошити ESP32")
    args = ap.parse_args()

    prior = load_params()

    # ---- Дані ----
    if args.file:
        raw = np.load(args.file, allow_pickle=True)["trials"].item()
        trials = raw
        src = args.file
    elif args.simulate:
        import copy

        truth = copy.deepcopy(prior)
        truth.omega0_sq *= 0.93
        truth.kappa, truth.d_visc, truth.c_coul, truth.tau_act = 0.9, 0.15, 0.06, 0.008
        trials = record_synthetic(truth)
        src = "synthetic"
        args.no_save = True
    else:
        trials = record_from_stand(args.port, args.baud)
        DATA_DIR.mkdir(exist_ok=True)
        src = DATA_DIR / f"sysid_{datetime.now():%Y%m%d_%H%M%S}.npz"
        np.savez_compressed(src, trials=np.array(trials, dtype=object))
        print(f"✓ Сирі дані збережено: {src}")

    # ---- Перевірка на незалежному тріалі ----
    print("\n\033[1;33m>>> Фіт на тріалах 1+2, перевірка на тріалі 3 ...\033[0m")
    val = None
    if 3 in trials and 1 in trials and 2 in trials:
        g_cv, _, loc_cv = fit(trials, [1, 2], prior)
        val = score(trials, g_cv, loc_cv, 3)
    if val:
        print(f"   прогноз тріалу 3: RMS {val[0]:.2f}°, R² {val[1]:.1f}%")

    # ---- Фінальний фіт на всіх ----
    print("\033[1;33m>>> Фінальний фіт на всіх тріалах ...\033[0m")
    g, std, locs = fit(trials, sorted(trials), prior)
    scores = {k: score(trials, g, locs, k) for k in sorted(trials)}

    print("\n" + "=" * 72)
    print(f" {'параметр':<12}{'значення':>12}{'± σ':>10}   {'було':>10}")
    for name, val_, s in zip(GLOBAL_NAMES, g, std):
        print(f" {name:<12}{val_:>12.5f}{s:>10.5f}   {getattr(prior, name):>10.5f}")
    print(f" ω0 = {np.sqrt(g[0]):.3f} рад/с (період {2 * np.pi / np.sqrt(g[0]):.3f} с)")
    for k, (rms, r2, _, loc) in scores.items():
        print(f" тріал {k}: RMS {rms:.2f}°, R² {r2:.1f}%, зсув нуля {np.degrees(loc[0]):+.2f}°")
    print("=" * 72)

    if g[1] < 0.9 or g[1] > 1.1:
        print(f"\033[33m⚠ κ = {g[1]:.2f}: візок рухається {'менше' if g[1] < 1 else 'більше'} за команду "
              "(пропуск кроків / прослизання паса / неточний STEPS_PER_METER).\033[0m")

    val_note = f"  — ПРОГНОЗ (не брав участі у фіті): RMS {val[0]:.2f}°" if val else ""
    plot(trials, g, locs, scores, val_note,
         DATA_DIR / "synthetic_check.png" if args.simulate else PLOT_PATH)

    good = all(s[0] < 3.0 for s in scores.values()) and (val is None or val[0] < 4.0)
    if not good and not args.force:
        print("\033[31m✗ Якість фіту низька (RMS > 3°) — параметри НЕ збережено. "
              "Перевірте графік; --force щоб зберегти все одно.\033[0m")
        return 1
    if args.no_save:
        print("(--no-save / --simulate: параметри не змінено)")
        return 0

    p = prior
    for name, v in zip(GLOBAL_NAMES, g):
        setattr(p, name, float(v))
    p.sysid_info = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "source": str(src),
        "rms_deg": {str(k): round(s[0], 3) for k, s in scores.items()},
        "validation_rms_deg": round(val[0], 3) if val else None,
        "std": {n: float(s) for n, s in zip(GLOBAL_NAMES, std)},
    }
    save_params(p)
    write_c_header(p)
    print("✓ common/pendulum_params.json + pendulum_params.h оновлено")

    if not args.no_traj:
        import mpc_generator

        mpc_generator.generate(p, verbose=False)
        print("✓ Траєкторію підйому перераховано (common/swingup_trajectory.h)")

    print("\n>>> Перевірка прошивки в симуляції з новими параметрами ...")
    rc = subprocess.run([sys.executable, str(ROOT / "single_pendulum" / "selftest.py"), "--quick"]).returncode
    if rc != 0:
        print("\033[31m✗ Самоперевірка не пройшла — не прошивайте, розберіться з параметрами.\033[0m")
        return 1

    if args.flash:
        exp = find_idf_export()
        if not exp:
            print("ESP-IDF не знайдено (задайте IDF_PATH).")
            return 1
        cmd = f"source '{exp}' >/dev/null 2>&1 && idf.py -C esp32_cart_pole -p {args.port} build flash"
        return subprocess.run(cmd, shell=True, executable="/bin/bash", cwd=str(ROOT)).returncode
    print("Прошити: idf.py -C esp32_cart_pole -p", args.port, "build flash   (або sys_id.py --flash)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
