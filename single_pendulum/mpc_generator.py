"""
mpc_generator.py
================
Генератор оптимальної траєкторії підйому (Swing-Up) одинарного перевернутого маятника
через пряму колокацію / нелінійну оптимізацію CasADi + IPOPT.

Враховує суворі фізичні обмеження приводу:
- a_max = 15.0 м/с² (тяга u_max = (M + m) * a_max)
- v_max = 0.66 м/с  (лінійна швидкість візка)
- x_rail = 0.5 м    (межа рейки)
- L = 0.35 м        (довжина стержня)
та зберігає отриману опорну траєкторію у файл 'mpc_trajectory.npz'.
"""

import os
import sys
from pathlib import Path
from typing import Optional
import numpy as np

# Налаштування каталогу кешу matplotlib
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

try:
    import casadi as ca
    try:
        ca.GlobalOptions.setNumpyMode(-1)
    except Exception:
        pass
except ImportError:
    ca = None

try:
    from .config import SinglePendulumConfig, DEFAULT_CONFIG
except ImportError:
    from config import SinglePendulumConfig, DEFAULT_CONFIG


def generate_single_pendulum_trajectory(
    cfg: Optional[SinglePendulumConfig] = None,
    output_path: Optional[Path] = None,
    T: float = 3.6,
    N: int = 180,
    verbose: bool = True
) -> str:
    """
    Розраховує оптимальну траєкторію розгойдування та підйому маятника з theta=pi (низ) до theta=0 (верх).
    """
    if ca is None:
        raise RuntimeError("Бібліотека casadi не встановлена. Запустіть скрипт у відповідному venv!")

    if cfg is None:
        cfg = DEFAULT_CONFIG

    if output_path is None:
        output_path = Path(__file__).resolve().parent / "mpc_trajectory.npz"
    else:
        output_path = Path(output_path)

    if verbose:
        print("=" * 70)
        print("Генерація оптимальної траєкторії MPC Swing-Up (Одинарний маятник)...")
        print(f"Параметри: L={cfg.L} м, M={cfg.M} кг, m={cfg.m} кг, g={cfg.g} м/с²")
        print(f"Обмеження: a_max={cfg.a_max} м/с², v_max={cfg.v_max} м/с, x_rail={cfg.x_rail} м")
        print(f"Тертя: b_cart={cfg.b_cart} Н*с/м, b_pend={cfg.b_pend} Н*м*с/рад")
        print("=" * 70)

    dt = T / N
    opti = ca.Opti()

    # Змінні стану: X = [x, theta, v, omega]
    X = opti.variable(4, N + 1)
    # Змінна керування: U = сила на візок (Н)
    U = opti.variable(1, N)

    pos = X[0, :]
    th = X[1, :]
    v = X[2, :]
    w = X[3, :]

    M_val = cfg.M
    m_val = cfg.m
    l_val = cfg.l
    g_val = cfg.g
    b_c = cfg.b_cart
    b_p = cfg.b_pend
    u_max_val = cfg.u_max
    v_max_val = cfg.v_max
    x_max_val = cfg.x_rail * 0.85  # Запас 15% до відбійників рейки

    def f_dyn(st, u_val):
        x_v, th_v, v_v, w_v = st[0], st[1], st[2], st[3]
        sin_t = ca.sin(th_v)
        cos_t = ca.cos(th_v)
        denom = l_val * (4.0 / 3.0 - m_val * (cos_t ** 2) / (M_val + m_val))
        temp = (u_val - b_c * v_v + m_val * l_val * (w_v ** 2) * sin_t) / (M_val + m_val)
        theta_acc = (g_val * sin_t - cos_t * temp - (b_p / (m_val * l_val)) * w_v) / denom
        x_acc = temp - (m_val * l_val * theta_acc * cos_t) / (M_val + m_val)
        return ca.vertcat(v_v, w_v, x_acc, theta_acc)

    # Інтегрування фізики через Рунге-Кутту 4-го порядку (RK4)
    for k in range(N):
        st_k = X[:, k]
        u_k = U[0, k]
        k1 = f_dyn(st_k, u_k)
        k2 = f_dyn(st_k + 0.5 * dt * k1, u_k)
        k3 = f_dyn(st_k + 0.5 * dt * k2, u_k)
        k4 = f_dyn(st_k + dt * k3, u_k)
        st_next = st_k + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        opti.subject_to(X[:, k + 1] == st_next)

    # Початковий стан: нерухомий у нижній точці (theta = pi)
    opti.subject_to(X[0, 0] == 0.0)
    opti.subject_to(X[1, 0] == np.pi)
    opti.subject_to(X[2, 0] == 0.0)
    opti.subject_to(X[3, 0] == 0.0)

    # Кінцевий цільовий стан: строго вертикально вгору (theta = 0), візок у центрі (x = 0)
    opti.subject_to(X[0, -1] == 0.0)
    opti.subject_to(X[1, -1] == 0.0)
    opti.subject_to(X[2, -1] == 0.0)
    opti.subject_to(X[3, -1] == 0.0)

    # Фізичні межі рейки, швидкості та тяги мотора
    opti.subject_to(opti.bounded(-x_max_val, pos, x_max_val))
    opti.subject_to(opti.bounded(-v_max_val, v, v_max_val))
    opti.subject_to(opti.bounded(-u_max_val, U, u_max_val))

    # Критерій оптимізації: мінімум енергетичних зусиль + утримання візка біля центру
    cost = (
        ca.sumsqr(U) * 0.005 * dt +
        ca.sumsqr(pos) * 5.0 * dt +
        ca.sumsqr(v) * 0.5 * dt
    )
    opti.minimize(cost)

    # Налаштування IPOPT
    nlpsol_opts = {
        'ipopt.print_level': 5 if verbose else 0,
        'ipopt.sb': 'no' if verbose else 'yes',
        'ipopt.max_iter': 1500,
        'ipopt.tol': 1e-6,
        'print_time': 0
    }
    opti.solver('ipopt', nlpsol_opts)

    # Початкове наближення (ініціалізація змінних)
    for k in range(N + 1):
        opti.set_initial(th[k], np.pi * (1.0 - k / N))
        opti.set_initial(pos[k], 0.0)
        opti.set_initial(v[k], 0.0)
        opti.set_initial(w[k], 0.0)

    sol = opti.solve()

    time_arr = np.linspace(0.0, T, N + 1)
    u_vals = np.append(sol.value(U), 0.0)  # додаємо 0 в кінцевий момент
    pos_vals = sol.value(pos)
    th_vals = sol.value(th)
    v_vals = sol.value(v)
    w_vals = sol.value(w)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        str(output_path),
        time=time_arr,
        u=u_vals,
        pos=pos_vals,
        theta=th_vals,
        v=v_vals,
        omega=w_vals,
        dt=dt,
        total_time=T,
        L=cfg.L,
        a_max=cfg.a_max,
        v_max=cfg.v_max
    )

    if verbose:
        print("\n" + "=" * 70)
        print("✓ ТРАЄКТОРІЮ УСПІШНО ЗГЕНЕРОВАНО ТА ЗБЕРЕЖЕНО!")
        print(f"Файл: {output_path}")
        print(f"Тривалість: {T:.2f} с ({N} кроків, dt = {dt:.3f} с)")
        print(f"Макс. координата |x|: {np.max(np.abs(pos_vals)):.3f} м (межа: {x_max_val:.2f} м)")
        print(f"Макс. швидкість  |v|: {np.max(np.abs(v_vals)):.3f} м/с (межа: {v_max_val:.2f} м/с)")
        print(f"Макс. тяга       |u|: {np.max(np.abs(u_vals)):.3f} Н (межа: {u_max_val:.2f} Н)")
        print(f"Кінцевий кут: theta = {np.degrees(th_vals[-1]):.3f}° (ідеальна вертикаль 0.0°)")
        print("=" * 70 + "\n")

    return str(output_path)


if __name__ == "__main__":
    generate_single_pendulum_trajectory()
