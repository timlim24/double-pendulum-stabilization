import os
import sys

# Налаштування змінних оточення перед завантаженням важких пакетів
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import argparse
import numpy as np
import casadi as ca

try:
    ca.GlobalOptions.setNumpyMode(-1)
except Exception:
    pass

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from template_model import template_model
from template_mpc import template_mpc
from template_simulator import template_simulator
import do_mpc


def pendulum_bars(x, L1=0.5, L2=0.5):
    """
    Обчислення (x, y) координат двох ланок маятника для заданого стану x.
    x[0]: позиція візка
    x[1]: кут ланки 1 (theta_1)
    x[2]: кут ланки 2 (theta_2)
    """
    x = x.flatten()
    # Перша ланка
    line_1_x = np.array([
        x[0],
        x[0] + L1 * np.sin(x[1])
    ])
    line_1_y = np.array([
        0.0,
        L1 * np.cos(x[1])
    ])

    # Друга ланка
    line_2_x = np.array([
        line_1_x[1],
        line_1_x[1] + L2 * np.sin(x[2])
    ])
    line_2_y = np.array([
        line_1_y[1],
        line_1_y[1] + L2 * np.cos(x[2])
    ])

    line_1 = np.stack((line_1_x, line_1_y))
    line_2 = np.stack((line_2_x, line_2_y))
    return line_1, line_2


def main():
    parser = argparse.ArgumentParser(description="do-mpc Double Inverted Pendulum (DIP) Simulation")
    parser.add_argument("--steps", type=int, default=100, help="Кількість кроків симуляції (за замовчуванням: 100)")
    parser.add_argument("--no-gif", action="store_true", help="Не генерувати gif-анімацію")
    parser.add_argument("--show", action="store_true", help="Показати інтерактивне вікно Matplotlib")
    parser.add_argument("--verbose", action="store_true", help="Виводити детальні логи ітерацій IPOPT")
    args = parser.parse_args()

    print("=" * 65)
    print("  do-mpc: Double Inverted Pendulum on a Cart (DAE + MPC)")
    print("=" * 65)

    # 1. Створення моделі, MPC-контролера та симулятора
    print("1. Ініціалізація моделі та компонентів...")
    model = template_model()
    mpc = template_mpc(model, silence_solver=not args.verbose)
    simulator = template_simulator(model)
    estimator = do_mpc.estimator.StateFeedback(model)

    # 2. Початковий стан: маятник майже повністю опущений вниз (0.99*pi)
    simulator.x0['theta'] = 0.99 * np.pi
    x0 = simulator.x0.cat.full()

    mpc.x0 = x0
    estimator.x0 = x0

    print("2. Розрахунок початкового наближення (warm start)...")
    mpc.set_initial_guess()

    # 3. Основний цикл замкненої симуляції
    n_steps = args.steps
    print(f"3. Запуск замкненої симуляції на {n_steps} кроків (dt = 0.04 с, t_total = {n_steps * 0.04:.2f} с)...")

    for k in range(n_steps):
        u0 = mpc.make_step(x0)
        y_next = simulator.make_step(u0)
        x0 = estimator.make_step(y_next)

        if (k + 1) % 10 == 0 or k == n_steps - 1:
            print(f"   [Крок {k + 1:3d}/{n_steps}] force = {float(u0[0, 0]):+6.3f} N | theta1 = {float(x0[1, 0]):+5.2f} rad | theta2 = {float(x0[2, 0]):+5.2f} rad | pos = {float(x0[0, 0]):+5.2f} m")

    print("\n✓ Симуляція успішно завершена!")

    # 4. Побудова графіків
    print("\n4. Побудова графіків телеметрії...")
    plt.ioff()
    fig = plt.figure(figsize=(15, 8))

    ax1 = plt.subplot2grid((4, 2), (0, 0), rowspan=4)
    ax2 = plt.subplot2grid((4, 2), (0, 1))
    ax3 = plt.subplot2grid((4, 2), (1, 1))
    ax4 = plt.subplot2grid((4, 2), (2, 1))
    ax5 = plt.subplot2grid((4, 2), (3, 1))

    ax2.set_ylabel(r'$E_{kin}$ [J]', fontsize=11)
    ax3.set_ylabel(r'$E_{pot}$ [J]', fontsize=11)
    ax4.set_ylabel('Кути [рад]', fontsize=11)
    ax5.set_ylabel('Сила [Н]', fontsize=11)
    ax5.set_xlabel('Час [с]', fontsize=11)

    for ax in [ax2, ax3, ax4, ax5]:
        ax.yaxis.set_label_position("right")
        ax.yaxis.tick_right()
        ax.grid(True, linestyle='--', alpha=0.6)
        if ax != ax5:
            ax.xaxis.set_ticklabels([])

    # Лінії do-mpc
    mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
    mpc_graphics.add_line(var_type='_aux', var_name='E_kin', axis=ax2)
    mpc_graphics.add_line(var_type='_aux', var_name='E_pot', axis=ax3)
    mpc_graphics.add_line(var_type='_x', var_name='theta', axis=ax4)
    mpc_graphics.add_line(var_type='_u', var_name='force', axis=ax5)

    # Ліва площина: візуалізація маятника
    ax1.axhline(0, color='black', linewidth=1.5)
    bar1, = ax1.plot([], [], '-o', color='#1f77b4', linewidth=6, markersize=10, label='Ланка 1')
    bar2, = ax1.plot([], [], '-o', color='#ff7f0e', linewidth=5, markersize=8, label='Ланка 2')
    cart_dot, = ax1.plot([], [], 's', color='#2ca02c', markersize=14, label='Візок')

    ax1.set_xlim(-1.8, 1.8)
    ax1.set_ylim(-1.2, 1.2)
    ax1.set_aspect('equal')
    ax1.legend(loc='upper right')
    ax1.set_title("Подвійний перевернутий маятник (DIP)", fontsize=13)

    fig.tight_layout()

    # Оновлення кінцевого положення
    line1, line2 = pendulum_bars(x0)
    bar1.set_data(line1[0], line1[1])
    bar2.set_data(line2[0], line2[1])
    cart_dot.set_data([x0[0, 0]], [0.0])

    mpc_graphics.plot_results()
    mpc_graphics.reset_axes()

    output_png = "dip_results.png"
    fig.savefig(output_png, dpi=150)
    print(f"✓ Графік збережено: {output_png}")

    # 5. Створення GIF-анімації
    if not args.no_gif:
        gif_path = "anim_dip.gif"
        print(f"5. Генерація GIF-анімації ({gif_path})...")

        x_arr = mpc.data['_x']

        def update(t_ind):
            l1, l2 = pendulum_bars(x_arr[t_ind])
            bar1.set_data(l1[0], l1[1])
            bar2.set_data(l2[0], l2[1])
            cart_dot.set_data([x_arr[t_ind, 0]], [0.0])
            mpc_graphics.plot_results(t_ind)
            mpc_graphics.plot_predictions(t_ind)
            mpc_graphics.reset_axes()
            return bar1, bar2, cart_dot

        anim = FuncAnimation(fig, update, frames=n_steps, repeat=False)
        writer = PillowWriter(fps=20)
        anim.save(gif_path, writer=writer)
        print(f"✓ Анімацію успішно збережено: {gif_path}")

    if args.show:
        plt.show()

    print("\nРоботу завершено успішно!")


if __name__ == '__main__':
    main()

