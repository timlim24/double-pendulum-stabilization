#!/usr/bin/env python3
"""
cli_control.py
==============
Панель керування та моніторингу стенду (ESP32 або virtual_stand.py).

Клавіші:
  [S]          — підйом (траєкторія + TVLQR, далі LQR-баланс)
  [X] / Space  — стоп: скасувати баланс/підйом, повернутися до гасіння
  [H]          — хомінг (калібрування X по стоперу)
  [C]          — прийняти поточне місце за X = 0 (якщо стопери недоступні)
  [I]          — автономний SysID (краще запускати через sys_id.py)
  [Q]          — вихід

    python cli_control.py [--port /dev/ttyUSB0] [--baud 115200]
"""

import argparse
import re
import select
import sys
import termios
import threading
import time
import tty

import serial

KEYS = {
    "s": (b"s", "\033[1;35mпідйом\033[0m"),
    "x": (b"x", "\033[1;31mстоп\033[0m"),
    " ": (b"x", "\033[1;31mстоп\033[0m"),
    "h": (b"h", "\033[1;33mхомінг\033[0m"),
    "c": (b"c", "\033[1;34mцентр = тут\033[0m"),
    "i": (b"i", "\033[1;36mSysID\033[0m"),
}

TELEMETRY_RE = re.compile(
    r"\[(?P<state>[^\]]*)\]\s*X:\s*(?P<x>[+-]?[\d.]+)m\s*\|\s*V:\s*(?P<v>[+-]?[\d.]+)m/s\s*\|\s*"
    r"L:(?P<l>\d)\s*R:(?P<r>\d)\s*\|\s*Кут:\s*(?P<th>[+-]?[\d.]+)°\s*\(вниз:\s*(?P<bot>[+-]?[\d.]+)°\)\s*\|\s*"
    r"w:\s*(?P<w>[+-]?[\d.]+)(?P<rest>.*)"
)
STATE_COLORS = {"БАЛАНС": "1;32", "ПІДЙОМ": "1;35", "ЗАНУЛЕННЯ": "1;34", "ОЧІКУЄ": "1;33", "SYSID": "1;36"}

running = True
last_cmd = "—"
lock = threading.Lock()


def keyboard_thread(ser):
    global running, last_cmd
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while running:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1).lower()
            if ch in ("q", "\x03"):
                running = False
                break
            if ch in KEYS:
                data, label = KEYS[ch]
                ser.write(data)
                ser.flush()
                with lock:
                    last_cmd = label
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    global running
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        sys.exit(f"Помилка відкриття порту {args.port}: {e}")

    w = sys.stdout.write
    w("\033[2J\033[H\033[1;36m" + "=" * 88 + "\033[0m\r\n")
    w("\033[1;32m  CART-POLE: ПАНЕЛЬ КЕРУВАННЯ\033[0m   "
      "[S] підйом  [X/Space] стоп  [H] хомінг  [C] центр=тут  [I] SysID  [Q] вихід\r\n")
    w("\033[1;36m" + "=" * 88 + "\033[0m\r\n")
    sys.stdout.flush()

    threading.Thread(target=keyboard_thread, args=(ser,), daemon=True).start()
    in_dump = False
    try:
        while running:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("SYSID_BEGIN"):
                in_dump = True
                w(f"\r\033[K\033[36m{line} (вивантаження даних SysID — для фіту використовуйте sys_id.py)\033[0m\r\n")
                continue
            if line.startswith("SYSID_END"):
                in_dump = False
                continue
            if in_dump or line.startswith("D,"):
                continue
            if line.startswith("EVT:"):
                text = line.split(" ", 1)[1] if " " in line else line
                w(f"\r\033[K{time.strftime('%H:%M:%S')}  {text}\r\n")
                sys.stdout.flush()
                continue
            m = TELEMETRY_RE.search(line)
            if not m:
                w(f"\r\033[K{line}\r\n")
                sys.stdout.flush()
                continue
            st = m.group("state")
            color = next((c for k, c in STATE_COLORS.items() if k in st), "1;37")
            l_badge = "\033[1;31mL:СТОП\033[0m" if m.group("l") == "0" else "\033[32mL:віль\033[0m"
            r_badge = "\033[1;31mR:СТОП\033[0m" if m.group("r") == "0" else "\033[32mR:віль\033[0m"
            with lock:
                cmd = last_cmd
            w(
                f"\r\033[K[\033[{color}m{st:<16}\033[0m] X:{float(m.group('x')) * 100:+6.1f}см "
                f"V:{float(m.group('v')):+5.2f} {l_badge} {r_badge} "
                f"θ:{float(m.group('th')):+6.1f}° (низ {float(m.group('bot')):+6.1f}°) "
                f"ω:{float(m.group('w')):+5.2f} | {cmd}"
            )
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        ser.close()
        w("\r\n\033[1;32mКонсоль закрито.\033[0m\r\n")


if __name__ == "__main__":
    main()
