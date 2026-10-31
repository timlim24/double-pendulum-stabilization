"""
mpc_generator.py
================
Генератор оптимальної траєкторії підйому (Swing-Up) одинарного перевернутого маятника
через пряму колокацію / нелінійну оптимізацію CasADi + IPOPT.

Враховує суворі фізичні параметри та обмеження реального стенду:
- Маса візка M = 0.130 кг, маса маятника m = 0.148 кг
- Довжина стержня L = 0.35 м, центр мас l = 0.300 м, інерція I = 0.0147 кг*м²
- Максимальне прискорення a_max = 7.50 м/с²
- Максимальна швидкість v_max = 0.70 м/с
- Межа ходу рейки |x| <= 0.18 м (18 см при доступних 27 см)
Зберігає отриману опорну траєкторію у:
1) 'mpc_trajectory.npz'
2) 'esp32_cart_pole/main/mpc_trajectory_data.h' (готовий масив С для ESP32).
"""

import os
import sys
from pathlib import Path
from typing import Optional

import numpy as np

# Налаштування каталогу кешу matplotlib
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

try:
    import casadi as ca

    try:
        ca.GlobalOptions.setNumpyMode(-1)
    except Exception:
        pass
except ImportError:
    ca = None

try:
    from .config import DEFAULT_CONFIG, SinglePendulumConfig
except ImportError:
    from config import DEFAULT_CONFIG, SinglePendulumConfig


def export_to_c_header(
    npz_path: Path,
    header_path: Path,
    time_arr: np.ndarray,
    pos_arr: np.ndarray,
    v_arr: np.ndarray,
    th_arr: np.ndarray,
    w_arr: np.ndarray,
    dt: float,
    total_time: float,
):
    """Експортує розраховану траєкторію у C-заголовок для ESP32."""
    header_path.parent.mkdir(parents=True, exist_ok=True)
    n_points = len(time_arr)

    with open(header_path, "w", encoding="utf-8") as f:
        f.write("// Auto-generated from single_pendulum/mpc_trajectory.npz\n")
        f.write("#pragma once\n\n")
        f.write(f"#define MPC_TRAJ_POINTS {n_points}\n")
        f.write(f"#define MPC_TRAJ_DT {dt:.4f}f\n")
        f.write(f"#define MPC_TRAJ_TOTAL_TIME {total_time:.2f}f\n\n")
        f.write("typedef struct {\n")
        f.write("    float time_s;\n")
        f.write("    float pos_m;\n")
        f.write("    float vel_mps;\n")
        f.write("    float theta_rad;\n")
        f.write("    float omega_radps;\n")
        f.write("} mpc_point_t;\n\n")
        f.write(f"static const mpc_point_t s_mpc_trajectory[MPC_TRAJ_POINTS] = {{\n")

        for k in range(n_points):
            f.write(
                f"    {{ {time_arr[k]:.3f}f, {pos_arr[k]:.6f}f, {v_arr[k]:.6f}f, {th_arr[k]:.6f}f, {w_arr[k]:.6f}f }},\n"
            )

        f.write("};\n")
    print(f"✓ C-заголовок для ESP32 збережено: {header_path}")


def generate_single_pendulum_trajectory(
    cfg: Optional[SinglePendulumConfig] = None,
    output_path: Optional[Path] = None,
    T: float = 4.2,
    N: int = 210,
    verbose: bool = True,
) -> str:
    """
    Розраховує оптимальну траєкторію розгойдування та підйому маятника з theta=pi (низ) до theta=0 (верх).
    """
    if ca is None:
        raise RuntimeError(
            "Бібліотека casadi не встановлена. Запустіть скрипт у відповідному venv!"
        )

    if cfg is None:
        cfg = DEFAULT_CONFIG

    if output_path is None:
        output_path = Path(__file__).resolve().parent / "mpc_trajectory.npz"
    else:
        output_path = Path(output_path)

    header_path = (
        Path(__file__).resolve().parent.parent
        / "esp32_cart_pole"
        / "main"
        / "mpc_trajectory_data.h"
    )

    if verbose:
        print("=" * 70)
        print("Генерація оптимальної траєкторії MPC Swing-Up під реальне залізо...")
        print(
            f"Параметри: L={cfg.L} м, l_com={cfg.l} м, I={cfg.I:.4f} кг*м^2, M={cfg.M} кг, m={cfg.m} кг"
        )
        print(
            f"Обмеження: a_max={cfg.a_max} м/с², v_max={cfg.v_max} м/с, x_rail={cfg.x_rail} м"
        )
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
    I_val = cfg.I
    g_val = cfg.g
    b_c = cfg.b_cart
    b_p = cfg.b_pend
    v_max_val = cfg.v_max
    a_max_val = cfg.a_max
    x_max_val = cfg.x_rail

    def f_dyn(st, u_val):
        x_v, th_v, v_v, w_v = st[0], st[1], st[2], st[3]
        sin_t = ca.sin(th_v)
        cos_t = ca.cos(th_v)
        temp = (u_val - b_c * v_v + m_val * l_val * (w_v**2) * sin_t) / (M_val + m_val)
        denom = (I_val / (m_val * l_val)) - (m_val * l_val * (cos_t**2)) / (
            M_val + m_val
        )
        theta_acc = (
            g_val * sin_t - cos_t * temp - (b_p / (m_val * l_val)) * w_v
        ) / denom
        x_acc = temp - (m_val * l_val * theta_acc * cos_t) / (M_val + m_val)
        return ca.vertcat(v_v, w_v, x_acc, theta_acc), x_acc

    # Інтегрування фізики через Рунге-Кутту 4-го порядку (RK4)
    x_acc_constraints = []
    for k in range(N):
        st_k = X[:, k]
        u_k = U[0, k]
        (k1, a1) = f_dyn(st_k, u_k)
        (k2, a2) = f_dyn(st_k + 0.5 * dt * k1, u_k)
        (k3, a3) = f_dyn(st_k + 0.5 * dt * k2, u_k)
        (k4, a4) = f_dyn(st_k + dt * k3, u_k)
        st_next = st_k + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        opti.subject_to(X[:, k + 1] == st_next)
        x_acc_constraints.append(a1)

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

    # Фізичні межі рейки, швидкості та прискорення візка
    opti.subject_to(opti.bounded(-x_max_val, pos, x_max_val))
    opti.subject_to(opti.bounded(-v_max_val, v, v_max_val))
    for a_c in x_acc_constraints:
        opti.subject_to(opti.bounded(-a_max_val, a_c, a_max_val))

    # Критерій оптимізації: мінімум енергетичних зусиль + утримання візка біля центру
    cost = (
        ca.sumsqr(U) * 0.005 * dt + ca.sumsqr(pos) * 5.0 * dt + ca.sumsqr(v) * 0.5 * dt
    )
    opti.minimize(cost)

    # Налаштування IPOPT
    nlpsol_opts = {
        "ipopt.print_level": 5 if verbose else 0,
        "ipopt.sb": "no" if verbose else "yes",
        "ipopt.max_iter": 1500,
        "ipopt.tol": 1e-4,
        "print_time": 0,
    }
    opti.solver("ipopt", nlpsol_opts)

    # Початкове наближення (ініціалізація змінних)
    for k in range(N + 1):
        opti.set_initial(th[k], np.pi * (1.0 - k / N))
        opti.set_initial(pos[k], 0.0)
        opti.set_initial(v[k], 0.0)
        opti.set_initial(w[k], 0.0)

    sol = opti.solve()

    time_arr = np.linspace(0.0, T, N + 1)
    u_vals = np.append(sol.value(U), 0.0)
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
        v_max=cfg.v_max,
    )

    export_to_c_header(
        output_path, header_path, time_arr, pos_vals, v_vals, th_vals, w_vals, dt, T
    )

    if verbose:
        print("\n" + "=" * 70)
        print("✓ ТРАЄКТОРІЮ УСПІШНО ЗГЕНЕРОВАНО ТА ЗБЕРЕЖЕНО!")
        print(f"Файл: {output_path}")
        print(f"Тривалість: {T:.2f} с ({N} кроків, dt = {dt:.3f} с)")
        print(
            f"Макс. координата |x|: {np.max(np.abs(pos_vals)):.3f} м (межа: {x_max_val:.2f} м)"
        )
        print(
            f"Макс. швидкість  |v|: {np.max(np.abs(v_vals)):.3f} м/с (межа: {v_max_val:.2f} м/с)"
        )
        print(
            f"Кінцевий кут: theta = {np.degrees(th_vals[-1]):.3f}° (ідеальна вертикаль 0.0°)"
        )
        print("=" * 70 + "\n")

    return str(output_path)


if __name__ == "__main__":
    generate_single_pendulum_trajectory()
