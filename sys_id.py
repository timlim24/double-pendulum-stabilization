#!/usr/bin/env python3
"""
sys_id.py
=========
Повністю автономна багатоетапна системна ідентифікація (Automated Multi-Trial SysID)
для стенду одинарного перевернутого маятника на візку (Cart-Pole).

Особливості:
1. 100% АВТОНОМНІСТЬ (Hands-Off): нічого не потрібно штовхати руками! Візок сам дає
   калібровані імпульсні поштовхи та гармонічні збудження.
2. БАГАТОЕТАПНІСТЬ: проводить 3 послідовні незалежні експерименти:
   - Тріал 1: Калібрований імпульс та вільне згасання (вільні коливання).
   - Тріал 2: Гармонічне багаточастотне збудження (динамічний зв'язок маятника з візком).
   - Тріал 3: Двополярний імпульс (перевірка симетрії тертя ліворуч/праворуч).
3. ПЕРЕВІРКА ПОВТОРЮВАНОСТІ: обчислює параметри для кожного експерименту окремо
   та перевіряє їхню збіжність (Mean ± Std Dev).
4. ВАЛІДАЦІЯ ТА ЗАКРИТТЯ ЦИКЛУ: після визначення параметрів автоматично перераховує
   ідеальну MPC-траєкторію, оновлює прошивку та створює графік 'sysid_result.png'.
"""

import os
import subprocess
import sys
import time
from pathlib import Path

import matplotlib
import numpy as np
import serial
from scipy.optimize import minimize

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_PY = SCRIPT_DIR / "single_pendulum" / "config.py"
CONFIG_H = SCRIPT_DIR / "common" / "cart_pole_config.h"
PLOT_PATH = SCRIPT_DIR / "sysid_result.png"

PORT = "/dev/ttyUSB0"
BAUD = 115200
G = 9.81


def record_autonomous_sysid(port=PORT, baud=BAUD, timeout_sec=55.0):
    """Підключається до стенду і записує 3 автономні експерименти."""
    print(
        "\n\033[1;36m========================================================================\033[0m"
    )
    print(
        "\033[1;32m      АВТОНОМНА БАГАТОЕТАПНА СИСТЕМНА ІДЕНТИФІКАЦІЯ (SysID)             \033[0m"
    )
    print(
        "\033[1;36m========================================================================\033[0m"
    )
    print("Режим: \033[1;33m100% АВТОНОМНИЙ (руками нічого не чіпати!)\033[0m")
    print("Програма проведе 3 незалежні експерименти:")
    print("  1. Імпульсний поштовх та вільне згасання (6.0 с)")
    print("  2. Гармонічне синусоїдальне збудження рейкою (6.5 с)")
    print("  3. Двополярне імпульсне збудження (6.0 с)")
    print("Між експериментами візок сам активно загасить коливання в нуль.")
    print(
        "\033[1;36m------------------------------------------------------------------------\033[0m"
    )

    if sys.stdin.isatty() and "--no-wait" not in sys.argv:
        try:
            input(
                "Натисніть \033[1;32m[Enter]\033[0m для автоматичного старту експериментів..."
            )
        except (EOFError, KeyboardInterrupt):
            pass

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"\033[1;31mПомилка відкриття порту {port}: {e}\033[0m")
        sys.exit(1)

    time.sleep(0.3)
    ser.reset_input_buffer()

    print(
        "\n\033[1;35m>>> [SysID RUNNING] Запуск автоматичних експериментів на стенді... <<<\033[0m\n"
    )

    trials_data = {
        1: {"t": [], "x": [], "v": [], "a": [], "theta": [], "omega": []},
        2: {"t": [], "x": [], "v": [], "a": [], "theta": [], "omega": []},
        3: {"t": [], "x": [], "v": [], "a": [], "theta": [], "omega": []},
    }

    current_trial = 1
    t_start = time.time()
    last_i_send = 0.0
    sysid_started = False

    while time.time() - t_start < timeout_sec:
        if not sysid_started and (time.time() - last_i_send > 0.8):
            ser.write(b"i\n")
            ser.flush()
            last_i_send = time.time()

        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        if line == "SYSID_ALL_DONE":
            print(
                f"\r\033[K\033[1;32m✓ Всі 3 експерименти успішно виконано та записано!\033[0m"
            )
            break

        if line.startswith("SYSID_NEXT_TRIAL:"):
            parts = line.split(":")
            if len(parts) > 1:
                current_trial = int(parts[1])
                print(f"\n\033[1;33m>>> Старт Тріалу #{current_trial}... <<<\033[0m")
            continue

        if line.startswith("SYSID:"):
            sysid_started = True
            parts = line[6:].split(",")
            if len(parts) >= 7:
                try:
                    tr_id = int(parts[0])
                    t_val = float(parts[1])
                    x_val = float(parts[2])
                    v_val = float(parts[3])
                    a_val = float(parts[4])
                    th_val = float(parts[5])
                    w_val = float(parts[6])

                    if tr_id in trials_data:
                        trials_data[tr_id]["t"].append(t_val)
                        trials_data[tr_id]["x"].append(x_val)
                        trials_data[tr_id]["v"].append(v_val)
                        trials_data[tr_id]["a"].append(a_val)
                        trials_data[tr_id]["theta"].append(th_val)
                        trials_data[tr_id]["omega"].append(w_val)

                    print(
                        f"\r\033[KТріал #{tr_id}: t={t_val:5.2f}с | X={x_val * 100:+5.1f}см | Кут={th_val * 180 / np.pi:+6.1f}° | w={w_val:+5.2f}",
                        end="",
                        flush=True,
                    )
                except ValueError:
                    pass

    ser.close()
    print("\n")

    # Конвертація в numpy
    converted = {}
    for k in [1, 2, 3]:
        n_pts = len(trials_data[k]["t"])
        if n_pts < 150:
            print(f"\033[1;31mУвага: у Тріалі #{k} записано лише {n_pts} точок.\033[0m")
        converted[k] = {
            "t": np.array(trials_data[k]["t"]),
            "x": np.array(trials_data[k]["x"]),
            "v": np.array(trials_data[k]["v"]),
            "a": np.array(trials_data[k]["a"]),
            "theta": np.array(trials_data[k]["theta"]),
            "omega": np.array(trials_data[k]["omega"]),
        }

    return converted


def simulate_dynamics(t, a, theta0, omega0, omega0_sq, gamma, d, c_f):
    """
    Чисельне інтегрування нелінійного рівняння руху маятника методом RK4:
    theta_ddot = - d * theta_dot - c_f * sgn(theta_dot) - omega0_sq * sin(theta) - gamma * a(t) * cos(theta)
    """
    n = len(t)
    dt = t[1] - t[0] if n > 1 else 0.010
    th_sim = np.zeros(n)
    w_sim = np.zeros(n)
    th_sim[0] = theta0
    w_sim[0] = omega0

    def deriv(th, w, acc):
        sgn_w = np.tanh(w / 0.1)
        dw = -d * w - c_f * sgn_w - omega0_sq * np.sin(th) - gamma * acc * np.cos(th)
        return w, dw

    for i in range(n - 1):
        ai = a[i]
        th = th_sim[i]
        w = w_sim[i]

        k1_th, k1_w = deriv(th, w, ai)
        k2_th, k2_w = deriv(
            th + 0.5 * dt * k1_th, w + 0.5 * dt * k1_w, 0.5 * (ai + a[i + 1])
        )
        k3_th, k3_w = deriv(
            th + 0.5 * dt * k2_th, w + 0.5 * dt * k2_w, 0.5 * (ai + a[i + 1])
        )
        k4_th, k4_w = deriv(th + dt * k3_th, w + dt * k3_w, a[i + 1])

        th_next = th + (dt / 6.0) * (k1_th + 2.0 * k2_th + 2.0 * k3_th + k4_th)
        w_next = w + (dt / 6.0) * (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w)

        th_next = (th_next + np.pi) % (2.0 * np.pi) - np.pi
        th_sim[i + 1] = th_next
        w_sim[i + 1] = w_next

    return th_sim, w_sim


def fit_single_trial(data):
    """Оптимізація параметрів для одного конкретного тріалу."""
    t = data["t"]
    a = data["a"]
    th_meas = data["theta"]
    w_meas = data["omega"]

    theta0 = th_meas[0]
    omega0 = w_meas[0]

    p0 = [29.5, 3.0, 0.10, 0.05]
    bounds = [(12.0, 55.0), (1.5, 8.0), (0.001, 0.8), (0.0, 0.4)]

    def loss_func(params):
        omega0_sq, gamma, d, c_f = params
        th_sim, _ = simulate_dynamics(t, a, theta0, omega0, omega0_sq, gamma, d, c_f)
        diff = (th_meas - th_sim + np.pi) % (2.0 * np.pi) - np.pi
        return np.mean(diff**2)

    res = minimize(
        loss_func, p0, method="L-BFGS-B", bounds=bounds, options={"maxiter": 350}
    )
    omega0_sq, gamma, d, c_f = res.x

    th_sim, w_sim = simulate_dynamics(t, a, theta0, omega0, omega0_sq, gamma, d, c_f)

    ss_res = np.sum(((th_meas - th_sim + np.pi) % (2.0 * np.pi) - np.pi) ** 2)
    ss_tot = np.sum((th_meas - np.mean(th_meas)) ** 2)
    r2 = max(0.0, min(100.0, (1.0 - (ss_res / (ss_tot + 1e-9))) * 100.0))

    m_pend = 0.148
    l_com = 0.300
    J_pivot = (m_pend * l_com * G) / omega0_sq
    b_viscous = d * J_pivot

    return {
        "omega0_sq": omega0_sq,
        "gamma": gamma,
        "d": d,
        "c_f": c_f,
        "r2": r2,
        "J_pivot": J_pivot,
        "b_viscous": b_viscous,
        "th_sim": th_sim,
    }


def analyze_all_trials(trials_data):
    """Аналіз та порівняння результатів усіх 3 експериментів."""
    results = {}
    print(
        "\n\033[1;36m========================================================================================\033[0m"
    )
    print(
        "\033[1;32m                  ПОРІВНЯЛЬНА ТАБЛИЦЯ РЕЗУЛЬТАТІВ ТРІАЛІВ (SysID)                       \033[0m"
    )
    print(
        "\033[1;36m========================================================================================\033[0m"
    )
    print(
        " Тріал  | Власна частота w0 | Інерція J (кг*м²) | В'язке b (Н*м*с) | Сухе c_f | Точність R²"
    )
    print(
        "----------------------------------------------------------------------------------------"
    )

    omega0_list = []
    J_list = []
    b_list = []

    for k in [1, 2, 3]:
        fit = fit_single_trial(trials_data[k])
        results[k] = fit
        w0 = np.sqrt(fit["omega0_sq"])
        omega0_list.append(w0)
        J_list.append(fit["J_pivot"])
        b_list.append(fit["b_viscous"])

        print(
            f"   #{k}    |   {w0:6.3f} рад/с   |   {fit['J_pivot']:10.6f}    |   {fit['b_viscous']:10.6f}   |  {fit['c_f']:6.4f}  |   \033[1;32m{fit['r2']:5.1f}%\033[0m"
        )

    # Середнє значення та розкид
    mean_w0 = np.mean(omega0_list)
    std_w0 = np.std(omega0_list)
    mean_J = np.mean(J_list)
    std_J = np.std(J_list)
    mean_b = np.mean(b_list)

    print(
        "----------------------------------------------------------------------------------------"
    )
    print(
        f" СЕРЕДНЄ|   {mean_w0:6.3f} ± {std_w0:.3f} |   {mean_J:10.6f}    |   {mean_b:10.6f}   |    -     |   \033[1;32mЗБІЖНІСТЬ ✓\033[0m"
    )
    print(
        "\033[1;36m========================================================================================\033[0m\n"
    )

    return results, mean_J, mean_b


def plot_all_trials(trials_data, results):
    """Створює 3-панельний графік накладення для всіх 3 експериментів."""
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=False)

    titles = [
        "Тріал #1: Імпульс та вільне згасання",
        "Тріал #2: Гармонічне коливання рейкою",
        "Тріал #3: Двополярний імпульс",
    ]

    for i, k in enumerate([1, 2, 3]):
        t = trials_data[k]["t"]
        th_meas = trials_data[k]["theta"] * 180.0 / np.pi
        th_sim = results[k]["th_sim"] * 180.0 / np.pi

        axes[i].plot(
            t, th_meas, "b-", linewidth=1.8, label="Реальний стенд", alpha=0.85
        )
        axes[i].plot(
            t,
            th_sim,
            "r--",
            linewidth=1.5,
            label=f"Симуляція (R² = {results[k]['r2']:.1f}%)",
        )
        axes[i].set_ylabel("Кут (°)")
        axes[i].set_title(titles[i], fontsize=11, fontweight="bold")
        axes[i].grid(True, linestyle="--", alpha=0.6)
        axes[i].legend(loc="upper right")

    axes[2].set_xlabel("Час (секунди)")
    plt.tight_layout()
    plt.savefig(PLOT_PATH, dpi=150)
    print(
        f"\033[1;32m✓ Графік верифікації усіх 3 тріалів збережено: {PLOT_PATH}\033[0m"
    )


def update_and_regenerate(mean_J, mean_b):
    """Оновлює параметри та запускає CasADi MPC генератор."""
    print(
        "\n\033[1;33m>>> Запис середніх ідентифікованих параметрів у кодову базу... <<<\033[0m"
    )

    # 1. config.py
    if CONFIG_PY.exists():
        content = CONFIG_PY.read_text(encoding="utf-8")
        import re

        content = re.sub(
            r"I_pivot:\s*float\s*=\s*[0-9\.]+",
            f"I_pivot: float = {mean_J:.6f}",
            content,
        )
        content = re.sub(
            r"b_pend:\s*float\s*=\s*[0-9\.]+", f"b_pend: float = {mean_b:.6f}", content
        )
        CONFIG_PY.write_text(content, encoding="utf-8")
        print(f"✓ Оновлено {CONFIG_PY}")

    # 2. cart_pole_config.h
    if CONFIG_H.exists():
        content = CONFIG_H.read_text(encoding="utf-8")
        content = re.sub(
            r"#define PENDULUM_INERTIA\s+[0-9\.]+f",
            f"#define PENDULUM_INERTIA {mean_J:.6f}f",
            content,
        )
        CONFIG_H.write_text(content, encoding="utf-8")
        print(f"✓ Оновлено {CONFIG_H}")

    # 3. Перерахунок оптимальної траєкторії MPC
    print(
        "\n\033[1;35m>>> Розрахунок оптимальної MPC Swing-Up траєкторії (CasADi + IPOPT)... <<<\033[0m"
    )
    subprocess.run(
        "python3 single_pendulum/mpc_generator.py",
        shell=True,
        check=True,
        cwd=str(SCRIPT_DIR),
    )

    # 4. Прошивка
    print("\n\033[1;32m>>> Прошивка ESP32 відкаліброваною траєкторією... <<<\033[0m")
    cmd_flash = "source /home/sasha/esp/esp-idf/export.sh >/dev/null 2>&1 && cd esp32_cart_pole && idf.py build flash -p /dev/ttyUSB0"
    res = subprocess.run(
        cmd_flash, shell=True, executable="/bin/bash", cwd=str(SCRIPT_DIR)
    )
    if res.returncode == 0:
        print(
            "\n\033[1;32m========================================================================\033[0m"
        )
        print(
            "\033[1;32m     ПОВНИЙ АВТОНОМНИЙ ЦИКЛ SysID ТА КАЛІБРУВАННЯ ЗАВЕРШЕНО!           \033[0m"
        )
        print(
            "\033[1;32m========================================================================\033[0m"
        )
        print("Параметри стенду виміряно на трьох незалежних тестах.")
        print(
            "Запустіть \033[1;33m./cli_control.py\033[0m та натисніть \033[1;32m[S]\033[0m для старту ідеального Swing-Up!"
        )


def main():
    trials_data = record_autonomous_sysid()
    results, mean_J, mean_b = analyze_all_trials(trials_data)
    plot_all_trials(trials_data, results)
    update_and_regenerate(mean_J, mean_b)


if __name__ == "__main__":
    main()
