#!/usr/bin/env python3
"""
virtual_stand.py
================
Віртуальний стенд: прошивка (C-ядро) + модель стенду, підключені до
псевдотерміналу. Говорить по "UART" рівно тим самим протоколом, що
esp32_cart_pole/main/main.c, тож cli_control.py та sys_id.py працюють з ним
так само, як із залізом.

    python single_pendulum/virtual_stand.py          # друкує шлях /dev/pts/N
    python cli_control.py --port /dev/pts/N
    python sys_id.py --port /dev/pts/N --no-save
"""

import argparse
import math
import os
import pty
import select
import time
import tty

try:
    from . import controller_lib as cl
    from .hwconfig import HW
    from .params import load_params
except ImportError:
    import controller_lib as cl
    from hwconfig import HW
    from params import load_params

KEYS = {
    "s": cl.CMD_SWINGUP, "S": cl.CMD_SWINGUP,
    "x": cl.CMD_ABORT, "X": cl.CMD_ABORT, " ": cl.CMD_ABORT,
    "h": cl.CMD_HOME, "H": cl.CMD_HOME,
    "c": cl.CMD_SET_CENTER, "C": cl.CMD_SET_CENTER,
    "i": cl.CMD_SYSID, "I": cl.CMD_SYSID,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=1.0, help="прискорення часу (1 = реальний час)")
    ap.add_argument("--omega-scale", type=float, default=1.0, help="справжній ω0² ×k (розбіжність моделі)")
    args = ap.parse_args()

    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    print(f"Віртуальний стенд: {os.ttyname(slave)}   (Ctrl+C — вихід)", flush=True)

    p = load_params()
    p.omega0_sq *= args.omega_scale
    bench = cl.Bench(p, x0=0.07, enc_off=777)
    seen = 0
    pending = 0
    dump = None  # (trial, rows)
    dt = cl.CTRL_DT
    decim = int(HW["SYSID_LOG_DECIM"])
    t_wall0 = time.monotonic()
    io_tick = 0

    def out(s):
        # Як UART на ESP32: блокуюче, без втрат рядків
        data = (s + "\r\n").encode()
        while data:
            try:
                n = os.write(master, data)
                data = data[n:]
            except BlockingIOError:
                select.select([], [master], [], 0.5)

    try:
        while True:
            # Команди
            try:
                for ch in os.read(master, 256).decode(errors="ignore"):
                    pending |= KEYS.get(ch, 0)
            except BlockingIOError:
                pass

            # 10 мс керування (5 тактів) за один прохід
            for _ in range(5):
                bench.step(pending, buffer_free=dump is None)
                pending = 0
            for _, code, text in bench.log_events[seen:]:
                out(f"EVT:{code} {text}")
                if code == cl.EV_SYSID_TRIAL_DONE:
                    trial = bench.ctrl.telemetry()["sysid_trial"]  # ще не збільшений
                    dump = (trial, sorted(bench.sysid.get(trial, [])))
            seen = len(bench.log_events)

            if dump is not None:
                trial, rows = dump
                out(f"SYSID_BEGIN,{trial},{len(rows)},{dt * decim:.4f}")
                for i, x, v, ticks in rows:
                    out(f"D,{i},{x:.5f},{v:.4f},{ticks}")
                out(f"SYSID_END,{trial}")
                bench.sysid.pop(trial, None)
                dump = None

            io_tick += 1
            if io_tick >= 10:  # 10 Гц телеметрія
                io_tick = 0
                tel = bench.ctrl.telemetry()
                pl = bench.plant.get()
                out(
                    f"[{cl.state_name(tel['state'])}] X:{tel['x']:+6.3f}m | V:{tel['v']:+5.2f}m/s | "
                    f"L:{0 if pl['stop_l'] else 1} R:{0 if pl['stop_r'] else 1} | "
                    f"Кут:{math.degrees(tel['theta']):+6.1f}° (вниз:{math.degrees(tel['phi']):+6.1f}°) | "
                    f"w:{tel['omega']:+5.2f} | Тіки:{pl['ticks']} Нуль:{tel['enc_zero']} | "
                    f"Кроки:0/0 | {'home OK' if tel['homed'] else 'НЕ КАЛІБР.'} | цикл:0мкс"
                )

            # Темп реального часу
            target = (bench.t / args.speed) - (time.monotonic() - t_wall0)
            if target > 0:
                time.sleep(target)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
