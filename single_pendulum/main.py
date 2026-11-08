"""
main.py
=======
Інтерактивний симулятор стенду (Pygame). Під капотом — ТОЙ САМИЙ C-код
керування, що прошитий в ESP32, і модель стенду з параметрами з SysID.
Тому поведінка в симуляторі = поведінка на залізі (з точністю до моделі).

Клавіші:
  S        — підйом (працює в стані "ОЧІКУЄ 's'")
  X/Space  — стоп (скасувати баланс/підйом)
  H        — хомінг          C — поточне місце = центр
  I        — SysID           R — перезапуск стенду
  ← / →    — поштовх маятника     M — розбіжність моделі (ω0² ±5%)
  F11      — повний екран    Esc — вихід
"""

import math

import numpy as np
import pygame

try:
    from . import controller_lib as cl
    from .hwconfig import HW
    from .simulation_manager import SimulationManager
    from .ui import Button, Oscilloscope, ProgressBar, Slider
except ImportError:
    import controller_lib as cl
    from hwconfig import HW
    from simulation_manager import SimulationManager
    from ui import Button, Oscilloscope, ProgressBar, Slider

ROD_LEN_M = 0.35
COM_M = 0.30
SWING_T = None
try:
    SWING_T = float(np.load(__import__("pathlib").Path(__file__).with_name("swingup_trajectory.npz"))["t"][-1])
except Exception:  # pragma: no cover
    SWING_T = 5.0

STATE_COLORS = {
    cl.ST_BALANCING: (70, 220, 130),
    cl.ST_SWINGUP: (190, 80, 230),
    cl.ST_ZEROING: (70, 170, 240),
    cl.ST_WAITING: (240, 190, 60),
    cl.ST_SYSID: (90, 210, 220),
}


class SinglePendulumApp:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Цифровий двійник стенду: одинарний маятник (прошивка ESP32 у симуляції)")
        info = pygame.display.Info()
        self.width = min(1500, max(1000, int(info.current_w * 0.8)))
        self.height = min(900, max(640, int(info.current_h * 0.8)))
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()
        self.fullscreen = False
        self.sim = SimulationManager()
        self.model_scales = [1.0, 0.95, 1.05]
        self.model_idx = 0
        self.osc = Oscilloscope(0, 0, 300, 130)
        self.progress = ProgressBar(30, 130, 400, 16)
        self._layout()

    # ------------------------------------------------------------------
    def _layout(self):
        s = max(0.85, min(self.width / 1300.0, self.height / 780.0))
        self.scale = s
        fam = "DejaVu Sans,Ubuntu,Arial"
        self.font = pygame.font.SysFont(fam, max(13, int(15 * s)))
        self.font_title = pygame.font.SysFont(fam, max(17, int(20 * s)), bold=True)
        self.font_small = pygame.font.SysFont(fam, max(11, int(12 * s)))
        self.font_mono = pygame.font.SysFont("DejaVu Sans Mono,Consolas", max(12, int(14 * s)))

        panel_y = self.height - int(190 * s)
        top = int(105 * s)
        avail = panel_y - 10 - top  # місце по вертикалі для маятника вгору і вниз
        self.origin_x = self.width // 2
        self.rail_y = top + avail // 2
        self.ppm = int(min(self.width * 0.55 / HW["RAIL_DISTANCE_M"], (avail / 2 - 12) / ROD_LEN_M))
        self.osc.rect = pygame.Rect(self.width - 320, 20, 300, 130)
        self.panel_y = panel_y
        sw, gap = int(200 * s), int(22 * s)
        g = self.sim.get_gains()  # [k_x, k_th, k_v, k_w, k_i]
        old = {sl.label: sl.val for sl in getattr(self, "sliders", [])}

        def mk(col, row, lo, hi, val, label, color):
            x = 30 + col * (sw + gap)
            return Slider(x, panel_y + 40 + row * 55, sw, 10, lo, hi, old.get(label, val), label, color=color)

        self.sliders = [
            mk(0, 0, 15.0, 150.0, g[1], "K_θ (м/с² на рад)", (230, 80, 60)),
            mk(0, 1, 2.0, 25.0, g[3], "K_ω (м/с² на рад/с)", (235, 130, 45)),
            mk(1, 0, -60.0, 0.0, g[0], "K_x (м/с² на м)", (50, 150, 235)),
            mk(1, 1, -40.0, 0.0, g[2], "K_v (м/с² на м/с)", (60, 190, 160)),
        ]
        self._gain_i = g[4]

        bw, bh = int(140 * s), int(34 * s)
        bx = 30 + 2 * (sw + gap) + 10

        def btn(col, row, text, color, cb):
            c2 = tuple(min(255, v + 30) for v in color)
            return Button(bx + col * (bw + 10), panel_y + 22 + row * (bh + 10), bw, bh, text, color, c2, cb)

        self.buttons = [
            btn(0, 0, "▶ Підйом [S]", (45, 120, 210), lambda: self.sim.command(cl.CMD_SWINGUP)),
            btn(1, 0, "■ Стоп [X]", (150, 50, 50), lambda: self.sim.command(cl.CMD_ABORT)),
            btn(2, 0, "⌂ Хомінг [H]", (85, 95, 115), lambda: self.sim.command(cl.CMD_HOME)),
            btn(0, 1, "≈ SysID [I]", (35, 125, 125), lambda: self.sim.command(cl.CMD_SYSID)),
            btn(1, 1, "◎ Центр [C]", (35, 125, 95), lambda: self.sim.command(cl.CMD_SET_CENTER)),
            btn(2, 1, "↻ Скинути [R]", (85, 95, 115), self.sim.reset),
            btn(0, 2, "← поштовх", (90, 80, 140), lambda: self.sim.kick(0.8)),
            btn(1, 2, "поштовх →", (90, 80, 140), lambda: self.sim.kick(-0.8)),
            btn(2, 2, "Модель [M]", (110, 100, 60), self.toggle_model),
        ]

    def toggle_model(self):
        self.model_idx = (self.model_idx + 1) % len(self.model_scales)
        sc = self.model_scales[self.model_idx]
        self.sim.set_model_scale(sc)
        self.sim.events.append(f"Розбіжність моделі: справжній ω0² ×{sc:.2f}")

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        if self.fullscreen:
            info = pygame.display.Info()
            self.width, self.height = info.current_w, info.current_h
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
        else:
            self.width, self.height = 1240, 760
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self._layout()

    # ------------------------------------------------------------------
    def run(self):
        keymap = {
            pygame.K_s: cl.CMD_SWINGUP, pygame.K_x: cl.CMD_ABORT, pygame.K_SPACE: cl.CMD_ABORT,
            pygame.K_h: cl.CMD_HOME, pygame.K_c: cl.CMD_SET_CENTER, pygame.K_i: cl.CMD_SYSID,
        }
        running = True
        while running:
            dt = min(0.05, self.clock.tick(60) / 1000.0)
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.VIDEORESIZE and not self.fullscreen:
                    self.width, self.height = ev.w, ev.h
                    self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                    self._layout()
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key in (pygame.K_F11, pygame.K_f):
                        self.toggle_fullscreen()
                    elif ev.key in keymap:
                        self.sim.command(keymap[ev.key])
                    elif ev.key == pygame.K_r:
                        self.sim.reset()
                    elif ev.key == pygame.K_LEFT:
                        self.sim.kick(0.8)
                    elif ev.key == pygame.K_RIGHT:
                        self.sim.kick(-0.8)
                    elif ev.key == pygame.K_m:
                        self.toggle_model()
                for b in self.buttons:
                    b.handle_event(ev)
                for sl in self.sliders:
                    sl.handle_event(ev)

            kth, kw, kx, kv = (sl.val for sl in self.sliders)
            self.sim.set_gains([kx, kth, kv, kw, self._gain_i])
            self.sim.update(dt)

            tel = self.sim.telemetry
            # Осцилограф: відхилення від найближчої вертикалі (без стрибків ±180° унизу)
            dev = tel["theta"] if abs(tel["theta"]) < math.pi / 2 else tel["phi"]
            self.osc.push(math.degrees(dev), tel["x"] * 100.0)
            self._render(tel, self.sim.plant)
            pygame.display.flip()
        pygame.quit()

    # ------------------------------------------------------------------
    def _render(self, tel, plant):
        scr = self.screen
        scr.fill((16, 20, 28))
        scr.blit(self.font_title.render("ЦИФРОВИЙ ДВІЙНИК СТЕНДУ — ОДИНАРНИЙ МАЯТНИК", True, (240, 245, 255)), (30, 20))
        col = STATE_COLORS.get(tel["state"], (200, 200, 200))
        scr.blit(self.font.render(f"Стан: {self.sim.state_name}", True, col), (30, 52))
        p = self.sim.params
        model_txt = f"Модель: ω0={p.omega0:.3f} рад/с, κ={p.kappa:.2f}, d={p.d_visc:.3f}, c={p.c_coul:.3f}, τ={p.tau_act * 1000:.1f} мс"
        if self.model_scales[self.model_idx] != 1.0:
            model_txt += f"   | справжній ω0² ×{self.model_scales[self.model_idx]:.2f}"
        scr.blit(self.font_small.render(model_txt, True, (150, 165, 190)), (30, 80))
        if tel["state"] == cl.ST_SWINGUP:
            self.progress.draw(scr, min(1.0, tel["swing_t"] / SWING_T), self.font_small, "Підйом")

        # Рейка, стопери, межі
        ppm, oy, ox = self.ppm, self.rail_y, self.origin_x
        lim = HW["RAIL_CENTER_M"]
        l_px, r_px = int(ox - lim * ppm), int(ox + lim * ppm)
        pygame.draw.line(scr, (40, 48, 65), (l_px - 20, oy), (r_px + 20, oy), 10)
        pygame.draw.line(scr, (95, 110, 140), (l_px - 20, oy - 2), (r_px + 20, oy - 2), 2)
        for px in (l_px - 14, r_px):
            pygame.draw.rect(scr, (220, 50, 60), (px, oy - 24, 14, 48), border_radius=3)
        for xm, c in ((HW["X_SOFT_LIMIT_M"], (200, 90, 60)), (HW["X_DOWN_LIMIT_M"], (60, 120, 200))):
            for sgn in (-1, 1):
                px = int(ox + sgn * xm * ppm)
                pygame.draw.line(scr, c, (px, oy - 12), (px, oy + 12), 2)
        pygame.draw.line(scr, (60, 75, 100), (ox, oy - 8), (ox, oy + 8), 2)

        # Візок (справжнє положення) та оцінка контролера (тонка рамка)
        cw, ch = max(26, int(HW["CART_WIDTH_M"] * ppm)), max(18, int(HW["CART_WIDTH_M"] * ppm * 0.55))
        cx = int(ox + plant["x"] * ppm)
        pygame.draw.rect(scr, (45, 95, 160), (cx - cw // 2, oy - ch // 2, cw, ch), border_radius=4)
        ex = int(ox + tel["x"] * ppm)
        pygame.draw.rect(scr, (200, 220, 255), (ex - cw // 2, oy - ch // 2, cw, ch), 1, border_radius=4)

        # Маятник. Конвенція стенду: θ (від верху) > 0 — вантаж у бік −X.
        th = plant["phi"] + math.pi
        rod = int(ROD_LEN_M * ppm)
        com = int(COM_M * ppm)
        end = (int(cx - rod * math.sin(th)), int(oy - rod * math.cos(th)))
        bob = (int(cx - com * math.sin(th)), int(oy - com * math.cos(th)))
        pygame.draw.line(scr, (200, 215, 235), (cx, oy), end, 4)
        r = max(9, int(13 * self.scale))
        pygame.draw.circle(scr, (230, 140, 40), bob, r)
        pygame.draw.circle(scr, (255, 190, 100), bob, r - 2)
        pygame.draw.circle(scr, (30, 35, 45), (cx, oy), 6)

        # Телеметрія
        lines = [
            f"X:  {tel['x'] * 100:+7.2f} см (справжнє {plant['x'] * 100:+.2f})",
            f"V:  {tel['v']:+7.3f} м/с",
            f"a:  {tel['a']:+7.2f} м/с²",
            f"θ:  {math.degrees(tel['theta']):+7.1f}°  (від низу {math.degrees(tel['phi']):+.1f}°)",
            f"ω:  {tel['omega']:+7.2f} рад/с",
            f"Калібровано: {'так' if tel['homed'] else 'НІ'}",
        ]
        ix, iy = self.width - 330, 165
        pygame.draw.rect(scr, (20, 26, 38), (ix - 10, iy - 8, 320, len(lines) * 21 + 14), border_radius=6)
        for i, ln in enumerate(lines):
            scr.blit(self.font_mono.render(ln, True, (215, 230, 250)), (ix, iy + i * 21))

        # Журнал подій (такі самі повідомлення друкує прошивка)
        ey = self.panel_y - 6 * 17 - 8
        for i, ln in enumerate(list(self.sim.events)[-6:]):
            scr.blit(self.font_small.render(ln, True, (170, 185, 205)), (30, ey + i * 17))

        self.osc.draw(scr, self.font_small)
        for b in self.buttons:
            b.draw(scr, self.font)
        for sl in self.sliders:
            sl.draw(scr, self.font_small, self.font_mono)


if __name__ == "__main__":
    SinglePendulumApp().run()
