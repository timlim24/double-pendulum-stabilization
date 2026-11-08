"""
mpc_generator.py
================
Офлайн-оптимізація траєкторії підйому (Swing-Up) одинарного маятника
(CasADi + IPOPT) + розрахунок часозмінного LQR (TVLQR) для замкненого
відстеження траєкторії на залізі.

Модель кінематична (кроковий візок виконує задане прискорення):
    стан  [x, θ, v, ω, a],  керування — ривок j = da/dt
    θ'' = ω0² sinθ + γ a cosθ − d ω − c tanh(ω/ε)      (θ від верху)

Обмеження: |x| ≤ X_MAX, |v| ≤ V_MAX, |a| ≤ A_MAX, |j| ≤ J_MAX.
Старт: висить нерухомо внизу (θ = π). Фініш: вертикально вгору, візок у центрі.

Результат:
  * single_pendulum/swingup_trajectory.npz
  * common/swingup_trajectory.h (масив для ESP32 / C-ядра: опорний стан,
    прямий зв'язок a_ref і коефіцієнти TVLQR K(t))

На стенді закон керування:  a = a_ref(t) − K(t) · (s − s_ref(t)).
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

try:
    import casadi as ca
except ImportError:  # pragma: no cover
    ca = None

try:
    from .hwconfig import COMMON_DIR, HW
    from .params import COULOMB_EPS, PendulumParams, load_params, terminal_cost_4state
except ImportError:
    from hwconfig import COMMON_DIR, HW
    from params import COULOMB_EPS, PendulumParams, load_params, terminal_cost_4state

HERE = Path(__file__).resolve().parent
NPZ_PATH = HERE / "swingup_trajectory.npz"
HEADER_PATH = COMMON_DIR / "swingup_trajectory.h"

# Обмеження траєкторії (м'якші за апаратні межі, щоб був запас для зворотного зв'язку)
T_TOTAL = 5.0
X_MAX = 0.14
V_MAX = 0.50
A_MAX = 3.5
J_MAX = 50.0
OPT_DT = 0.02
STORE_DT = 0.01

# Ваги TVLQR [x, θ, v, ω], R
TV_Q = np.diag([80.0, 300.0, 8.0, 30.0])
TV_R = np.array([[0.5]])


def pendulum_acc(p: PendulumParams, th, w, a, sin=np.sin, cos=np.cos, tanh=np.tanh):
    return (
        p.omega0_sq * sin(th)
        + p.gamma * a * cos(th)
        - p.d_visc * w
        - p.c_coul * tanh(w / COULOMB_EPS)
    )


def optimize(p: PendulumParams, T=T_TOTAL, verbose=True):
    if ca is None:
        raise RuntimeError("casadi не встановлено: pip install casadi")
    N = int(round(T / OPT_DT))
    dt = T / N
    opti = ca.Opti()
    X = opti.variable(5, N + 1)  # x, θ, v, ω, a
    U = opti.variable(1, N)  # ривок

    def f(s, j):
        acc = pendulum_acc(p, s[1], s[3], s[4], ca.sin, ca.cos, ca.tanh)
        return ca.vertcat(s[2], s[3], s[4], acc, j)

    for k in range(N):
        s, j = X[:, k], U[0, k]
        k1 = f(s, j)
        k2 = f(s + dt / 2 * k1, j)
        k3 = f(s + dt / 2 * k2, j)
        k4 = f(s + dt * k3, j)
        opti.subject_to(X[:, k + 1] == s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

    opti.subject_to(X[:, 0] == ca.DM([0, np.pi, 0, 0, 0]))
    opti.subject_to(X[:, N] == ca.DM([0, 0, 0, 0, 0]))
    opti.subject_to(opti.bounded(-X_MAX, X[0, :], X_MAX))
    opti.subject_to(opti.bounded(-V_MAX, X[2, :], V_MAX))
    opti.subject_to(opti.bounded(-A_MAX, X[4, :], A_MAX))
    opti.subject_to(opti.bounded(-J_MAX, U, J_MAX))

    cost = dt * (ca.sumsqr(X[4, :]) + 0.002 * ca.sumsqr(U) + 20.0 * ca.sumsqr(X[0, :]))
    opti.minimize(cost)

    for k in range(N + 1):
        tt = k / N
        opti.set_initial(X[1, k], np.pi * (1 - tt))
        opti.set_initial(X[0, k], 0.05 * np.sin(2 * np.pi * tt * T * 0.94))

    opti.solver(
        "ipopt",
        {
            "ipopt.print_level": 3 if verbose else 0,
            "print_time": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": 3000,
        },
    )
    sol = opti.solve()
    return dt, np.array(sol.value(X)), np.atleast_1d(sol.value(U))


def resample_fine(p: PendulumParams, opt_dt, S, U, fine_dt):
    """
    Перераховує траєкторію з кроком контуру керування (ривок ZOH, RK4).
    Кожен інтервал оптимізатора стартує з його вузла (multiple shooting), бо
    наскрізне інтегрування біля нестійкої верхньої точки розбігається.
    """
    sub = int(round(opt_dt / fine_dt))
    h = opt_dt / sub

    def f(s, j):
        return np.array([s[2], s[3], s[4], pendulum_acc(p, s[1], s[3], s[4]), j])

    out = np.zeros((5, len(U) * sub + 1))
    for k, j in enumerate(U):
        s = S[:, k].copy()
        for i in range(sub):
            out[:, k * sub + i] = s
            k1 = f(s, j)
            k2 = f(s + h / 2 * k1, j)
            k3 = f(s + h / 2 * k2, j)
            k4 = f(s + h * k3, j)
            s = s + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    out[:, -1] = S[:, -1]
    return out


def tvlqr(p: PendulumParams, ref, dt):
    """Дискретний TVLQR уздовж опорної траєкторії. Повертає K[k] (4 коеф.)."""
    _, P = terminal_cost_4state(p, dt)
    n = ref.shape[1]
    K_all = np.zeros((n, 4))
    I4 = np.eye(4)
    for k in range(n - 1, -1, -1):
        th, w, a = ref[1, k], ref[3, k], ref[4, k]
        dfdth = p.omega0_sq * np.cos(th) - p.gamma * a * np.sin(th)
        dfdw = -p.d_visc - p.c_coul / COULOMB_EPS / np.cosh(w / COULOMB_EPS) ** 2
        Ac = np.array(
            [[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0], [0, dfdth, 0, dfdw]], dtype=float
        )
        Bc = np.array([[0], [0], [1], [p.gamma * np.cos(th)]], dtype=float)
        Ad = I4 + Ac * dt + Ac @ Ac * dt * dt / 2
        Bd = Bc * dt + Ac @ Bc * dt * dt / 2
        K = np.linalg.solve(TV_R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        P = TV_Q + Ad.T @ P @ (Ad - Bd @ K)
        P = 0.5 * (P + P.T)
        K_all[k] = K.ravel()
    return K_all


def export_header(t, ref, K, path=HEADER_PATH):
    n = len(t)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("// АВТОГЕНЕРОВАНО single_pendulum/mpc_generator.py — НЕ РЕДАГУВАТИ\n")
        fh.write("// Траєкторія підйому + TVLQR: a = a_ref - K·(s - s_ref), s=[x,θ,v,ω]\n")
        fh.write("#pragma once\n\n")
        fh.write(f"#define SWING_N {n}\n")
        fh.write(f"#define SWING_DT {STORE_DT:.4f}f\n")
        fh.write(f"#define SWING_T {t[-1]:.4f}f\n")
        fh.write(f"#define SWING_X_MAX {X_MAX:.3f}f\n")
        fh.write(f"#define SWING_V_MAX {V_MAX:.3f}f\n")
        fh.write(f"#define SWING_A_MAX {A_MAX:.3f}f\n\n")
        fh.write("typedef struct {\n  float x, th, v, w, a;\n  float kx, kth, kv, kw;\n} swing_point_t;\n\n")
        fh.write("static const swing_point_t s_swing_traj[SWING_N] = {\n")
        for i in range(n):
            x, th, v, w, a = ref[:, i]
            k = K[i]
            fh.write(
                f"  {{{x:.6f}f, {th:.6f}f, {v:.6f}f, {w:.6f}f, {a:.5f}f, "
                f"{k[0]:.4f}f, {k[1]:.4f}f, {k[2]:.4f}f, {k[3]:.4f}f}},\n"
            )
        fh.write("};\n")


def generate(p: PendulumParams = None, verbose=True, tvlqr_only=False):
    p = p or load_params()
    ctrl_dt = HW["CTRL_DT"]
    if tvlqr_only:
        fine = np.load(NPZ_PATH)["fine"]
        return _finish(p, fine, ctrl_dt, verbose)
    if verbose:
        print("=" * 70)
        print("Оптимізація траєкторії підйому (кінематична модель, CasADi/IPOPT)")
        print(f"ω0={p.omega0:.3f} рад/с, γ={p.gamma:.3f}, d={p.d_visc:.3f}, c={p.c_coul:.3f}")
        print(f"T={T_TOTAL} с, |x|≤{X_MAX} м, |v|≤{V_MAX} м/с, |a|≤{A_MAX} м/с², |j|≤{J_MAX} м/с³")
        print("=" * 70)
    opt_dt, S, U = optimize(p, verbose=verbose)
    fine = resample_fine(p, opt_dt, S, U, ctrl_dt)
    return _finish(p, fine, ctrl_dt, verbose)


def _finish(p, fine, ctrl_dt, verbose):
    K_fine = tvlqr(p, fine, ctrl_dt)

    step = int(round(STORE_DT / ctrl_dt))
    idx = np.arange(0, fine.shape[1], step)
    t = idx * ctrl_dt
    ref = fine[:, idx]
    K = K_fine[idx]

    np.savez_compressed(
        NPZ_PATH, t=t, x=ref[0], theta=ref[1], v=ref[2], omega=ref[3], a=ref[4], K=K,
        fine=fine,
    )
    export_header(t, ref, K)
    if verbose:
        end = fine[:, -1]
        print(f"✓ {NPZ_PATH}\n✓ {HEADER_PATH}")
        print(
            f"max|x|={np.abs(ref[0]).max():.3f} м  max|v|={np.abs(ref[2]).max():.3f} м/с  "
            f"max|a|={np.abs(ref[4]).max():.2f} м/с²  (крок ≤ {np.abs(ref[2]).max() * HW['STEPS_PER_METER'] / 1000:.1f} кГц)"
        )
        print(
            f"кінець (ресемпл 500 Гц): θ={np.degrees(end[1]):+.2f}°, ω={end[3]:+.3f}, x={end[0]:+.4f}"
        )
        print(f"max|K| = {np.abs(K).max(axis=0).round(1)}")
    return t, ref, K


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tvlqr-only", action="store_true",
                    help="не переоптимізовувати траєкторію, лише перерахувати TVLQR")
    generate(tvlqr_only=ap.parse_args().tvlqr_only)
    sys.exit(0)
