#!/usr/bin/env python3
"""
cli_control.py
==============
CLI панель керування та моніторингу стенду Inverted Pendulum.
Фіксований екран без тремтіння, зїжджання рядків чи нескінченного скролу!

Керування:
  [S] або [Enter] - Запуск MPC Swing-Up
  [C] або [0]     - Ручне занулення центру (X=0)
  [H]             - Пошук кінцевика (Homing)
  [I]             - Запуск автономного SysID
  [Q]             - Вихід
"""

import os
import re
import select
import sys
import termios
import threading
import time
import tty

import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

running = True
ser = None
status_lock = threading.Lock()
last_event_msg = "Готовий до роботи"


def clean_ansi(text):
    """Видаляє ANSI escape-послідовності для вирівнювання довжини."""
    ansi_escape = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
    return ansi_escape.sub("", text)


def keyboard_thread_func():
    """Потік миттєвого зчитування клавіш без затримок буфера."""
    global running, ser, last_event_msg
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while running:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                ch = sys.stdin.read(1)
                if not running:
                    break

                if ch in ["s", "S", "\r", "\n", " "]:
                    if ser and ser.is_open:
                        ser.write(b"s\n")
                        ser.flush()
                        with status_lock:
                            last_event_msg = (
                                "\033[1;35m[COMMAND] Старт Swing-Up ('s')\033[0m"
                            )
                elif ch in ["c", "C", "0"]:
                    if ser and ser.is_open:
                        ser.write(b"c\n")
                        ser.flush()
                        with status_lock:
                            last_event_msg = (
                                "\033[1;34m[COMMAND] Ручне занулення X=0 ('c')\033[0m"
                            )
                elif ch in ["h", "H"]:
                    if ser and ser.is_open:
                        ser.write(b"h\n")
                        ser.flush()
                        with status_lock:
                            last_event_msg = (
                                "\033[1;33m[COMMAND] Пошук нуля Homing ('h')\033[0m"
                            )
                elif ch in ["i", "I"]:
                    if ser and ser.is_open:
                        ser.write(b"i\n")
                        ser.flush()
                        with status_lock:
                            last_event_msg = "\033[1;36m[COMMAND] Старт автономного SysID ('i')\033[0m"
                elif ch in ["q", "Q", "\x03"]:
                    running = False
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    global running, ser, last_event_msg

    # Очищення екрана
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write(
        "\033[1;36m========================================================================================\033[0m\r\n"
    )
    sys.stdout.write(
        "\033[1;32m                     CART-POLE LIVE DASHBOARD & CONTROL CONSOLE                         \033[0m\r\n"
    )
    sys.stdout.write(
        "\033[1;36m========================================================================================\033[0m\r\n"
    )
    sys.stdout.write("Гарячі клавіші (миттєвий відгук):\r\n")
    sys.stdout.write(
        "  \033[1;33m[S] або [Enter]\033[0m - Запуск MPC Swing-Up підйому\r\n"
    )
    sys.stdout.write(
        "  \033[1;34m[C] або [0]\033[0m     - Ручне занулення центру (прийняти поточне місце за X=0)\r\n"
    )
    sys.stdout.write(
        "  \033[1;35m[H]\033[0m             - Пошук кінцевика (Homing)\r\n"
    )
    sys.stdout.write(
        "  \033[1;36m[I]\033[0m             - Запуск автономного SysID\r\n"
    )
    sys.stdout.write("  \033[1;31m[Q] або Ctrl+C\033[0m  - Вихід\r\n")
    sys.stdout.write(
        "\033[1;36m----------------------------------------------------------------------------------------\033[0m\r\n"
    )
    sys.stdout.flush()

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        sys.stdout.write(
            f"\r\n\033[1;31mПомилка відкриття порту {PORT}: {e}\033[0m\r\n"
        )
        sys.exit(1)

    kb_thread = threading.Thread(target=keyboard_thread_func, daemon=True)
    kb_thread.start()

    # Парсер рядка телеметрії стенду
    line_pattern = re.compile(
        r"\[(.*?)\]\s*X:\s*([+-]?[\d\.]+m)\s*\|\s*V:\s*([+-]?[\d\.]+m/s)\s*\|\s*L:(\d)\s*R:(\d)\s*\|\s*Кут:\s*([+-]?[\d\.]+°)\s*\((?:від низу|вниз):\s*([+-]?[\d\.]+°)\).*?\|\s*w:\s*([+-]?[\d\.]+)"
    )

    try:
        while running:
            raw_line = ser.readline().decode("utf-8", errors="ignore")
            if not raw_line:
                continue
            line = raw_line.strip()
            if not line:
                continue

            # Якщо це спеціальне системне повідомлення ESP-IDF (I, W, E або інше)
            if not line.startswith("[") or "]" not in line:
                sys.stdout.write(f"\r\033[K{line}\r\n")
                sys.stdout.flush()
                continue

            # Парсинг стандартного рядка телеметрії для фіксованого форматування
            m = line_pattern.search(clean_ansi(line))
            if m:
                state_str = m.group(1)[:16]
                x_str = m.group(2)
                v_str = m.group(3)
                l_val = m.group(4)
                r_val = m.group(5)
                th_str = m.group(6)
                bot_str = m.group(7)
                w_str = m.group(8)

                # Кольорове позначення стоперів: 0 = червоний [НАТИСНУТО], 1 = зелений [ВІЛЬНО]
                l_badge = (
                    "\033[1;31mL:СТОП\033[0m"
                    if l_val == "0"
                    else "\033[1;32mL:ВІЛЬ\033[0m"
                )
                r_badge = (
                    "\033[1;31mR:СТОП\033[0m"
                    if r_val == "0"
                    else "\033[1;32mR:ВІЛЬ\033[0m"
                )

                # Колір стану
                state_color = "\033[1;33m"
                if "БАЛАНС" in state_str:
                    state_color = "\033[1;32m"
                elif "SWING" in state_str:
                    state_color = "\033[1;35m"
                elif "ЗАНУЛЕННЯ" in state_str:
                    state_color = "\033[1;34m"
                elif "АВАРІЙ" in state_str:
                    state_color = "\033[1;31m"

                # Фіксоване однорядкове відображення без скролу та зсувів
                with status_lock:
                    evt = last_event_msg

                out = (
                    f"\r\033[K[{state_color}{state_str:<16}\033[0m] "
                    f"X:{x_str:>7} | V:{v_str:>8} | {l_badge} {r_badge} | "
                    f"Кут:{th_str:>7} (вниз:{bot_str:>6}) | w:{w_str:>5} | {evt}"
                )
                sys.stdout.write(out)
                sys.stdout.flush()
            else:
                sys.stdout.write(f"\r\033[K{line}\r\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        if ser:
            ser.close()
        sys.stdout.write("\r\n\033[1;32mКонсоль закрито.\033[0m\r\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
