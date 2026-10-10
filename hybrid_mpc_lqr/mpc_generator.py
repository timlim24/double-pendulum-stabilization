"""
mpc_generator.py
================
Генератор оптимальної траєкторії підйому (Swing-Up) маятника через do-mpc.
Прораховує оптимальний маневр підйому з нижнього спокійного положення (0.99*pi)
з урахуванням фізичних параметрів конфігурації (маса візка, тертя в шарнірах)
та зберігає отриману послідовність керувань і траєкторію стану
у стиснений файл 'mpc_trajectory.npz'.
"""

import os
import sys
from pathlib import Path
from typing import Optional
import numpy as np

# Налаштування змінних оточення для matplotlib перед завантаженням
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

try:
    import casadi as ca
    try:
        ca.GlobalOptions.setNumpyMode(-1)
    except Exception:
        pass
    import do_mpc
except ImportError as e:
    print(f"Помилка завантаження бібліотек do-mpc / casadi: {e}")
    do_mpc = None

try:
    from .config import SystemConfig, DEFAULT_CONFIG
except ImportError:
    from config import SystemConfig, DEFAULT_CONFIG


def build_mpc_model(cfg: SystemConfig):
    """
    Побудова неперервної DAE моделі подвійного перевернутого маятника
    з точним врахуванням геометрії, мас та тертя у підшипниках.
    """
    model = do_mpc.model.Model('continuous')

    m0 = cfg.m0
    m1 = cfg.m1
    m2 = cfg.m2
    L1 = cfg.L1
    L2 = cfg.L2
    g = cfg.g
    b_j1 = cfg.b_joint1
    b_j2 = cfg.b_joint2

    l1 = L1 / 2.0
    l2 = L2 / 2.0
    J1 = (m1 * l1**2) / 3.0
    J2 = (m2 * l2**2) / 3.0

    h1 = m0 + m1 + m2
    h2 = m1 * l1 + m2 * L1
    h3 = m2 * l2
    h4 = m1 * l1**2 + m2 * L1**2 + J1
    h5 = m2 * l2 * L1
    h6 = m2 * l2**2 + J2
    h7 = (m1 * l1 + m2 * L1) * g
    h8 = m2 * l2 * g

    pos = model.set_variable('_x', 'pos')
    theta = model.set_variable('_x', 'theta', (2, 1))
    dpos = model.set_variable('_x', 'dpos')
    dtheta = model.set_variable('_x', 'dtheta', (2, 1))

    u = model.set_variable('_u', 'force')

    ddpos = model.set_variable('_z', 'ddpos')
    ddtheta = model.set_variable('_z', 'ddtheta', (2, 1))

    model.set_rhs('pos', dpos)
    model.set_rhs('theta', dtheta)
    model.set_rhs('dpos', ddpos)
    model.set_rhs('dtheta', ddtheta)

    # Рівняння Ейлера-Лагранжа з демпфуванням у підшипниках
    euler_lagrange = ca.vertcat(
        h1 * ddpos + h2 * ddtheta[0] * ca.cos(theta[0]) + h3 * ddtheta[1] * ca.cos(theta[1])
        - (h2 * dtheta[0]**2 * ca.sin(theta[0]) + h3 * dtheta[1]**2 * ca.sin(theta[1]) + u),
        h2 * ca.cos(theta[0]) * ddpos + h4 * ddtheta[0] + h5 * ca.cos(theta[0] - theta[1]) * ddtheta[1]
        - (h7 * ca.sin(theta[0]) - h5 * dtheta[1]**2 * ca.sin(theta[0] - theta[1]) - b_j1 * dtheta[0]),
        h3 * ca.cos(theta[1]) * ddpos + h5 * ca.cos(theta[0] - theta[1]) * ddtheta[0] + h6 * ddtheta[1]
        - (h5 * dtheta[0]**2 * ca.sin(theta[0] - theta[1]) + h8 * ca.sin(theta[1]) - b_j2 * dtheta[1])
    )
    model.set_alg('euler_lagrange', euler_lagrange)

    E_kin_cart = 0.5 * m0 * dpos**2
    E_kin_p1 = 0.5 * m1 * ((dpos + l1 * dtheta[0] * ca.cos(theta[0]))**2 + (l1 * dtheta[0] * ca.sin(theta[0]))**2) + 0.5 * J1 * dtheta[0]**2
    E_kin_p2 = 0.5 * m2 * ((dpos + L1 * dtheta[0] * ca.cos(theta[0]) + l2 * dtheta[1] * ca.cos(theta[1]))**2 + (L1 * dtheta[0] * ca.sin(theta[0]) + l2 * dtheta[1] * ca.sin(theta[1]))**2) + 0.5 * J2 * dtheta[1]**2

    E_kin = E_kin_cart + E_kin_p1 + E_kin_p2
    E_pot = m1 * g * l1 * ca.cos(theta[0]) + m2 * g * (L1 * ca.cos(theta[0]) + l2 * ca.cos(theta[1]))

    model.set_expression('E_kin', E_kin)
    model.set_expression('E_pot', E_pot)
    model.setup()

    return model


def generate_and_save_trajectory(
    config: Optional[SystemConfig] = None,
    output_path: Optional[Path] = None,
    n_steps: int = 100,
    dt: float = 0.04,
    verbose: bool = True
) -> str:
    """
    Виконує симуляцію замкненого контуру do-mpc та зберігає траєкторію у файл .npz.
    """
    if do_mpc is None:
        raise RuntimeError("do-mpc або casadi недоступні у поточному середовищі Python.")

    cfg = config if config is not None else DEFAULT_CONFIG

    if output_path is None:
        output_path = Path(__file__).resolve().parent / "mpc_trajectory.npz"

    if verbose:
        print("=" * 65)
        print("Генерація оптимальної траєкторії Swing-Up через do-mpc...")
        print(f"Параметри: m0={cfg.m0} кг, b_cart={cfg.b_cart}, b_joint={cfg.b_joint1}")
        print("=" * 65)

    # 1. Побудова моделі та оптимізатора
    model = build_mpc_model(cfg)
    mpc = do_mpc.controller.MPC(model)

    nlpsol_opts = {
        'ipopt.linear_solver': 'mumps',
        'ipopt.print_level': 0,
        'ipopt.sb': 'yes',
        'print_time': 0
    }
    setup_mpc = {
        'n_horizon': 100,
        'n_robust': 0,
        'open_loop': 0,
        't_step': dt,
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 3,
        'collocation_ni': 1,
        'store_full_solution': True,
        'nlpsol_opts': nlpsol_opts
    }
    mpc.set_param(**setup_mpc)
    mpc.set_objective(mterm=model.aux['E_kin'] - model.aux['E_pot'], lterm=model.aux['E_kin'] - model.aux['E_pot'])
    mpc.set_rterm(force=0.1)

    f_max = 6.5
    mpc.bounds['lower', '_u', 'force'] = -f_max
    mpc.bounds['upper', '_u', 'force'] = f_max
    mpc.setup()

    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step=dt, integration_tool='collocation')
    simulator.setup()

    estimator = do_mpc.estimator.StateFeedback(model)

    # 2. Початковий стан: ідеальне нижнє вертикальне положення спокою (pi + 1e-4)
    simulator.x0['theta'] = np.pi + 1e-4
    x0 = simulator.x0.cat.full()

    mpc.x0 = x0
    estimator.x0 = x0
    mpc.set_initial_guess()

    time_hist = []
    u_hist = []
    x_hist = []
    current_t = 0.0

    if verbose:
        print(f"Розрахунок {n_steps} кроків MPC (dt = {dt} с, горизонт = {n_steps * dt:.2f} с)...")

    for k in range(n_steps):
        u0 = mpc.make_step(x0)
        u_val = float(u0[0, 0])

        time_hist.append(current_t)
        u_hist.append(u_val)
        x_hist.append(x0.flatten())

        y_next = simulator.make_step(u0)
        x0 = estimator.make_step(y_next)
        current_t += dt

        if verbose and ((k + 1) % 20 == 0 or k == n_steps - 1):
            th1_deg = np.degrees(float(x0[1, 0]))
            th2_deg = np.degrees(float(x0[2, 0]))
            print(f"  [Крок {k + 1:3d}/{n_steps}] u = {u_val:+6.3f} Н | th1 = {th1_deg:+6.1f}° | th2 = {th2_deg:+6.1f}°")

    time_hist.append(current_t)
    u_hist.append(0.0)
    x_hist.append(x0.flatten())

    time_arr = np.array(time_hist, dtype=np.float64)
    u_arr = np.array(u_hist, dtype=np.float64)
    x_mat = np.array(x_hist, dtype=np.float64)

    np.savez_compressed(
        str(output_path),
        time=time_arr,
        u=u_arr,
        pos=x_mat[:, 0],
        theta1=x_mat[:, 1],
        theta2=x_mat[:, 2],
        v=x_mat[:, 3],
        omega1=x_mat[:, 4],
        omega2=x_mat[:, 5],
        dt=dt,
        total_time=current_t
    )

    if verbose:
        print(f"\n✓ Траєкторію успішно розраховано та збережено у: {output_path}")
        print(f"  Кількість точок: {len(time_arr)}")
        print(f"  Кінцеві кути: th1 = {np.degrees(x_mat[-1, 1]):.2f}°, th2 = {np.degrees(x_mat[-1, 2]):.2f}°")

    return str(output_path)


if __name__ == "__main__":
    generate_and_save_trajectory()
