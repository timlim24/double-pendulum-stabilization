"""
main.py
=======
Головний модуль запуску інтерактивного додатку гібридної стабілізації
подвійного перевернутого маятника на візку (MPC + LQR) у Pygame.
Повністю підтримує адаптивне масштабування під високі роздільні здатності (2K, 4K, Full HD).
"""

import sys
import math
from pathlib import Path
from typing import Optional
import pygame
import numpy as np

try:
    from .config import SystemConfig, DEFAULT_CONFIG
    from .dynamics import wrap_to_pi
    from .simulation_manager import SimulationManager, ControlMode
    from .ui import Button, Slider, Oscilloscope, ProgressBar
except ImportError:
    from config import SystemConfig, DEFAULT_CONFIG
    from dynamics import wrap_to_pi
    from simulation_manager import SimulationManager, ControlMode
    from ui import Button, Slider, Oscilloscope, ProgressBar


class HybridDoublePendulumApp:
    """
    Головний клас додатку Pygame.
    Відповідає за адаптивне масштабування інтерфейсу (DPI-aware),
    обробку подій, фізичний цикл та високоякісний рендеринг сцени.
    """

    def __init__(self, config: SystemConfig = DEFAULT_CONFIG):
        self.cfg = config
        pygame.init()
        pygame.display.set_caption("Гібридна стабілізація подвійного маятника (MPC Swing-Up + LQR) — Pygame 60 FPS")

        # Автоматичне визначення роздільної здатності дисплея користувача
        info = pygame.display.Info()
        screen_w, screen_h = info.current_w, info.current_h

        # Підбір комфортного розміру вікна для 2K / 4K / Full HD
        if screen_w >= 2200:  # 2K (2560x1440) або 4K
            self.width = min(2000, int(screen_w * 0.78))
            self.height = min(1200, int(screen_h * 0.78))
        elif screen_w >= 1600:  # 1080p
            self.width = min(1500, int(screen_w * 0.80))
            self.height = min(920, int(screen_h * 0.80))
        else:  # Ноутбуки та менші екрани
            self.width = min(1240, max(800, screen_w - 60))
            self.height = min(760, max(600, screen_h - 80))

        self.is_fullscreen = False
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        # Менеджер симуляції
        self.sim = SimulationManager(self.cfg)

        # Стан взаємодії мишкою
        self.dragging_cart = False
        self.dragging_bob = False
        self.mouse_target_x = 0.0

        # Сила поштовху клавішами
        self.nudge_force = self.cfg.key_nudge_force

        # Стан утримання кнопок та клавіш поштовху
        self.holding_key_l = False
        self.holding_key_r = False
        self.holding_push_l = False
        self.holding_push_r = False

        # Коефіцієнт масштабу UI (за замовчуванням 0.50 — зменшено на 50% за запитом)
        self.ui_scale_user = 0.75

        # Осцилограф та прогрес-бар
        self.oscilloscope = Oscilloscope(self.width - 320, 20, 300, 130)
        self.progress_bar = ProgressBar(30, 130, 420, 16)

        # Оновлення геометрії сцени та створення масштабованого UI
        self._update_layout()

    def _do_kick(self, direction: float) -> None:
        """Здійснює ударний імпульс швидкості (Kick) та імпульсну силу для відчутного збурення візка."""
        ratio = max(0.2, min(2.5, self.nudge_force / max(1.0, self.cfg.key_nudge_force)))
        speed = self.cfg.nudge_kick_speed * ratio
        self.sim.apply_kick(direction=direction, speed_step=speed)
        self.sim.set_continuous_nudge(direction * self.nudge_force)

    def _update_layout(self) -> None:
        """
        Динамічно оновлює шрифти, геометрію кнопок, повзунків та сцени
        пропорційно до поточного розміру вікна.
        UI масштабується окремо та зменшено на 50% для максимального візуального простору маятника.
        """
        # Базове масштабування фізичної сцени (рейка, маятник, візок)
        self.scene_scale = max(0.85, min(self.width / 1300.0, self.height / 780.0))

        # Масштабування елементів інтерфейсу (50% від базового масштабу)
        self.ui_scale_user = getattr(self, 'ui_scale_user', 0.50)
        self.ui_scale = max(0.38, self.scene_scale * self.ui_scale_user)
        scale = self.ui_scale

        # Адаптивні компактні шрифти для інтерфейсу
        self.font_title = pygame.font.SysFont("dejavusans", max(11, int(22 * scale)), bold=True)
        self.font_badge = pygame.font.SysFont("dejavusans", max(11, int(23 * scale)), bold=True)
        self.font_main = pygame.font.SysFont("dejavusans", max(9, int(17 * scale)))
        self.font_small = pygame.font.SysFont("dejavusans", max(8, int(14 * scale)))
        self.font_button = pygame.font.SysFont("dejavusans", max(9, int(16 * scale)), bold=True)

        self.origin_x = self.width // 2
        self.rail_y = int(self.height * 0.44)

        # Масштаб пікселів на метр (PPM) для фізичної сцени
        available_w = max(600, self.width - int(120 * self.scene_scale))
        self.ppm = available_w / (2.0 * (self.cfg.X_rail + 0.35))

        self.cart_w_px = max(65, int(0.38 * self.ppm))
        self.cart_h_px = max(26, int(0.16 * self.ppm))

        # Оновлення осцилографа у верхньому правому кутку (компактний)
        ow = int(280 * scale)
        oh = int(115 * scale)
        self.oscilloscope.rect = pygame.Rect(self.width - ow - int(20 * scale), int(12 * scale), ow, oh)

        # Прогрес-бар
        bar_w = min(int(360 * scale), int(self.width * 0.32))
        bar_h = max(11, int(15 * scale))
        self.progress_bar.rect = pygame.Rect(int(24 * scale), int(106 * scale), bar_w, bar_h)

        # Нижня панель керування UI (компактна, на 50% менша за висотою)
        panel_h = int(170 * scale)
        panel_y = self.height - panel_h - int(12 * scale)
        panel_w = self.width - int(36 * scale)
        panel_x = int(18 * scale)

        self.panel_rect = pygame.Rect(panel_x, panel_y, panel_w, panel_h)

        # Розміри та кроки елементів керування
        slider_w = min(int(220 * scale), int(panel_w * 0.22))
        slider_h = max(7, int(10 * scale))
        col_gap = int(24 * scale)

        col1_x = panel_x + int(18 * scale)
        col2_x = col1_x + slider_w + col_gap
        col3_x = col2_x + slider_w + col_gap

        # Слайдери
        v_qth = getattr(self, 'slider_qth', None).val if hasattr(self, 'slider_qth') else self.cfg.q_th2
        v_qx = getattr(self, 'slider_qx', None).val if hasattr(self, 'slider_qx') else self.cfg.q_x
        v_fmax = getattr(self, 'slider_fmax', None).val if hasattr(self, 'slider_fmax') else self.cfg.F_max
        v_amax = getattr(self, 'slider_amax', None).val if hasattr(self, 'slider_amax') else self.cfg.a_max
        v_nudge = getattr(self, 'slider_nudge', None).val if hasattr(self, 'slider_nudge') else self.nudge_force

        s_y1 = panel_y + int(38 * scale)
        s_y2 = s_y1 + int(42 * scale)
        s_y3 = s_y2 + int(42 * scale)

        self.slider_qth = Slider(
            col1_x, s_y1, slider_w, slider_h, 100.0, 1800.0, v_qth,
            "LQR Q_θ (жорсткість кута)", color=(230, 80, 60),
            on_change=lambda val: self.sim.lqr.update_gains(q_th=val)
        )
        self.slider_qx = Slider(
            col1_x, s_y2, slider_w, slider_h, 0.0, 60.0, v_qx,
            "LQR Q_x (центрування візка)", color=(50, 150, 235),
            on_change=lambda val: self.sim.lqr.update_gains(q_pos=val)
        )
        self.slider_nudge = Slider(
            col1_x, s_y3, slider_w, slider_h, 5.0, 50.0, v_nudge,
            "Поштовх (Kick & Push)", unit=" Н", color=(225, 145, 40),
            on_change=lambda val: setattr(self, 'nudge_force', val)
        )

        self.slider_fmax = Slider(
            col2_x, s_y1, slider_w, slider_h, 10.0, 60.0, v_fmax,
            "F_max (макс сила)", unit=" Н", color=(185, 80, 220),
            on_change=lambda val: setattr(self.cfg, 'F_max', val)
        )
        self.slider_amax = Slider(
            col2_x, s_y2, slider_w, slider_h, 5.0, 40.0, v_amax,
            "a_max (макс прискорення)", unit=" м/с²", color=(45, 195, 175),
            on_change=lambda val: setattr(self.cfg, 'a_max', val)
        )

        self.sliders = [
            self.slider_qth, self.slider_qx, self.slider_nudge,
            self.slider_fmax, self.slider_amax
        ]

        # Кнопки дій
        btn_w = min(int(160 * scale), int(panel_w * 0.15))
        btn_h = max(24, int(40 * scale))
        btn_gap = int(10 * scale)
        btn_col1 = col3_x
        btn_col2 = btn_col1 + btn_w + btn_gap
        btn_col3 = btn_col2 + btn_w + btn_gap

        b_y1 = panel_y + int(34 * scale)
        b_y2 = b_y1 + int(48 * scale)

        self.btn_swing = Button(
            btn_col1, b_y1, btn_w, btn_h, "🚀 MPC Swing-Up",
            (30, 110, 195), (45, 140, 240), self.sim.trigger_mpc_swingup,
            border_radius=int(6 * scale)
        )
        self.btn_zero = Button(
            btn_col1, b_y2, btn_w, btn_h, "🎯 Занулити (LQR)",
            (25, 130, 130), (35, 165, 165), self.sim.trigger_down_zero,
            border_radius=int(6 * scale)
        )
        self.btn_motor = Button(
            btn_col2, b_y1, btn_w, btn_h, "⏻ Мотор: УВІМК",
            (40, 130, 85), (55, 165, 110), self._toggle_motor_btn,
            border_radius=int(6 * scale)
        )
        self.btn_reset = Button(
            btn_col2, b_y2, btn_w, btn_h, "↻ Скинути (Free)",
            (105, 110, 125), (130, 135, 150), lambda: self.sim.reset(start_down=True),
            border_radius=int(6 * scale)
        )

        half_w = (btn_w - int(6 * scale)) // 2
        self.btn_push_l = Button(
            btn_col3, b_y1, half_w, btn_h, "◄ Вліво",
            (145, 80, 50), (180, 105, 65),
            callback=lambda: self._do_kick(-1.0),
            on_press=lambda: setattr(self, 'holding_push_l', True),
            on_release=lambda: setattr(self, 'holding_push_l', False),
            border_radius=int(6 * scale)
        )
        self.btn_push_r = Button(
            btn_col3 + half_w + int(6 * scale), b_y1, half_w, btn_h, "Вправо ►",
            (145, 80, 50), (180, 105, 65),
            callback=lambda: self._do_kick(1.0),
            on_press=lambda: setattr(self, 'holding_push_r', True),
            on_release=lambda: setattr(self, 'holding_push_r', False),
            border_radius=int(6 * scale)
        )
        self.btn_shock = Button(
            btn_col3, b_y2, btn_w, btn_h, "⚡ Тест удару",
            (155, 60, 60), (190, 80, 80),
            callback=lambda: self._do_kick(1.8 if np.random.rand() > 0.5 else -1.8),
            border_radius=int(6 * scale)
        )

        # Кнопка перемикання повного екрана під осцилографом
        f_w = int(95 * scale)
        f_h = int(24 * scale)
        self.btn_fullscreen = Button(
            self.width - f_w - int(20 * scale), self.oscilloscope.rect.bottom + int(8 * scale), f_w, f_h, "⛶ Екран",
            (45, 55, 75), (65, 80, 105), self.toggle_fullscreen,
            border_radius=int(5 * scale)
        )

        self.buttons = [
            self.btn_swing, self.btn_zero, self.btn_motor, self.btn_reset,
            self.btn_push_l, self.btn_push_r, self.btn_shock, self.btn_fullscreen
        ]

    def _toggle_motor_btn(self) -> None:
        self.sim.toggle_motor()
        if self.sim.mode == ControlMode.MANUAL_OFF:
            self.btn_motor.text = "⏻ Мотор: ВИМК"
            self.btn_motor.color = (150, 45, 45)
            self.btn_motor.hover_color = (190, 60, 60)
        else:
            self.btn_motor.text = "⏻ Мотор: УВІМК"
            self.btn_motor.color = (40, 130, 85)
            self.btn_motor.hover_color = (55, 165, 110)

    def toggle_fullscreen(self) -> None:
        self.is_fullscreen = not self.is_fullscreen
        if self.is_fullscreen:
            info = pygame.display.Info()
            self.width, self.height = info.current_w, info.current_h
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
        else:
            info = pygame.display.Info()
            screen_w, screen_h = info.current_w, info.current_h
            if screen_w >= 2200:
                self.width, self.height = min(2000, int(screen_w * 0.78)), min(1200, int(screen_h * 0.78))
            else:
                self.width, self.height = min(1500, int(screen_w * 0.8)), min(920, int(screen_h * 0.8))
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self._update_layout()

    def run(self) -> None:
        """Головний цикл симуляції та подій."""
        running = True

        while running:
            # 60 FPS та вимірювання дельти часу
            raw_dt = self.clock.tick(self.cfg.fps) / 1000.0
            frame_dt = min(0.04, max(0.001, raw_dt))

            # 1. ОБРОБКА ПОДІЙ
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.width = event.w
                    self.height = event.h
                    self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                    self._update_layout()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key in (pygame.K_F11, pygame.K_f):
                        self.toggle_fullscreen()
                    elif event.key == pygame.K_SPACE:
                        self.sim.trigger_mpc_swingup()
                    elif event.key in (pygame.K_z, pygame.K_0):
                        self.sim.trigger_down_zero()
                    elif event.key == pygame.K_m:
                        self._toggle_motor_btn()
                    elif event.key == pygame.K_r:
                        self.sim.reset(start_down=True)
                    elif event.key == pygame.K_LEFT:
                        self._do_kick(-1.0)
                        self.holding_key_l = True
                    elif event.key == pygame.K_RIGHT:
                        self._do_kick(1.0)
                        self.holding_key_r = True
                    elif event.key == pygame.K_LEFTBRACKET:
                        self.ui_scale_user = max(0.25, self.ui_scale_user - 0.05)
                        self._update_layout()
                    elif event.key == pygame.K_RIGHTBRACKET:
                        self.ui_scale_user = min(1.20, self.ui_scale_user + 0.05)
                        self._update_layout()
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.ui_scale_user = max(0.25, self.ui_scale_user - 0.05)
                        self._update_layout()
                    elif event.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                        self.ui_scale_user = min(1.20, self.ui_scale_user + 0.05)
                        self._update_layout()
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_LEFT:
                        self.holding_key_l = False
                    elif event.key == pygame.K_RIGHT:
                        self.holding_key_r = False

                # Події елементів UI
                ui_consumed = False
                for btn in self.buttons:
                    if btn.handle_event(event):
                        ui_consumed = True
                for sld in self.sliders:
                    if sld.handle_event(event):
                        ui_consumed = True

                # Взаємодія мишкою з візком на сцені
                if not ui_consumed:
                    self._handle_mouse_interaction(event)

            # 2. ФІЗИЧНЕ ПЕРЕТЯГУВАННЯ ВІЗКА КУРСОРОМ ТА УТРИМАННЯ ТЯГИ
            if self.dragging_cart:
                self.sim.set_mouse_drag_target(self.mouse_target_x)

            # Безперервна тяга при затисканні стрілок клавіатури або кнопок миші
            cont_force = 0.0
            if self.holding_key_l or self.holding_push_l:
                cont_force -= self.nudge_force
            if self.holding_key_r or self.holding_push_r:
                cont_force += self.nudge_force
            if cont_force != 0.0:
                self.sim.set_continuous_nudge(cont_force)

            # 3. ФІЗИЧНИЙ КРОК (СУБ-ІНТЕГРУВАННЯ RK4)
            sub_steps = 8
            dt_sub = frame_dt / sub_steps
            for _ in range(sub_steps):
                self.sim.update(dt_sub)

            # 4. ЗАПИС У ТЕЛЕМЕТРІЮ ТА ОСЦИЛОГРАФ
            th1_deg = np.degrees(wrap_to_pi(self.sim.dynamics.state[1]))
            th2_deg = np.degrees(wrap_to_pi(self.sim.dynamics.state[2]))
            self.oscilloscope.add_sample(th1_deg, th2_deg)

            # 5. ВІДМАЛЬОВКА
            self._render()
            pygame.display.flip()

        pygame.quit()
        sys.exit()

    def _handle_mouse_interaction(self, event: pygame.event.Event) -> None:
        """Обробляє захоплення та перетягування візка або вантажу маятника курсором миші."""
        x_c_px = int(self.origin_x + self.sim.dynamics.state[0] * self.ppm)
        cart_rect = pygame.Rect(
            x_c_px - self.cart_w_px // 2,
            self.rail_y - self.cart_h_px // 2,
            self.cart_w_px,
            self.cart_h_px
        )

        cart_pt, joint_pt, tip_pt = self.sim.dynamics.get_cartesian_points()
        p2 = (int(self.origin_x + tip_pt[0] * self.ppm), int(self.rail_y - tip_pt[1] * self.ppm))

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            if cart_rect.collidepoint(mx, my):
                self.dragging_cart = True
                self.mouse_target_x = (mx - self.origin_x) / self.ppm
            elif math.hypot(mx - p2[0], my - p2[1]) < max(20, int(26 * self.scene_scale)):
                self.dragging_bob = True
                # Якщо тягнемо маятник руками, скидаємо режим розгону MPC
                if self.sim.mode == ControlMode.MPC_SWINGUP:
                    self.sim.mpc.stop()
                    self.sim.mode = ControlMode.FALLEN
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging_cart = False
            self.dragging_bob = False
            self.sim.release_mouse_drag()
        elif event.type == pygame.MOUSEMOTION:
            mx, my = event.pos
            if self.dragging_cart:
                target_m = (mx - self.origin_x) / self.ppm
                self.mouse_target_x = float(np.clip(target_m, -self.cfg.X_rail, self.cfg.X_rail))
            elif self.dragging_bob:
                # Пряме підтягування кутів маятника за курсором
                dx = (mx - x_c_px) / self.ppm
                dy = (self.rail_y - my) / self.ppm
                th = math.atan2(dx, dy)
                self.sim.dynamics.state[1] = th
                self.sim.dynamics.state[2] = th
                self.sim.dynamics.state[4] = 0.0
                self.sim.dynamics.state[5] = 0.0

    def _render(self) -> None:
        """Малює сцену симулятора та адаптивний графічний інтерфейс."""
        self.screen.fill((16, 20, 28))

        # 1. Рейка та мітки
        self._render_rail()

        # 2. Візок та зв'язок з мишкою
        self._render_cart()

        # 3. Ланки подвійного маятника
        self._render_pendulum()

        # 4. Вектор сили мотора
        self._render_force_vector()

        # 5. Інтерфейсні панелі та текст
        self._render_hud()
        self._render_controls()

    def _render_rail(self) -> None:
        """Відмальовує напрямну рейку та відбійники з урахуванням масштабу сцени."""
        scale = self.scene_scale
        rail_start_x = int(self.origin_x - (self.cfg.X_rail + 0.25) * self.ppm)
        rail_end_x = int(self.origin_x + (self.cfg.X_rail + 0.25) * self.ppm)

        r_thickness = max(6, int(10 * scale))
        pygame.draw.line(self.screen, (40, 48, 65), (rail_start_x, self.rail_y), (rail_end_x, self.rail_y), r_thickness)
        pygame.draw.line(self.screen, (90, 105, 135), (rail_start_x, self.rail_y - 2), (rail_end_x, self.rail_y - 2), max(2, int(3 * scale)))

        # Відбійники
        bump_l_x = int(self.origin_x - self.cfg.X_rail * self.ppm)
        bump_r_x = int(self.origin_x + self.cfg.X_rail * self.ppm)
        b_h = max(24, int(36 * scale))
        b_w = max(12, int(18 * scale))

        for bx in [bump_l_x, bump_r_x]:
            pygame.draw.line(self.screen, (220, 55, 65), (bx, self.rail_y - b_h // 2), (bx, self.rail_y + b_h // 2), max(3, int(5 * scale)))
            sign = 1 if bx == bump_l_x else -1
            bump_rect = pygame.Rect(bx - (b_w if sign == 1 else 0), self.rail_y - b_h // 3, b_w, int(b_h * 0.66))
            pygame.draw.rect(self.screen, (175, 40, 50), bump_rect, border_radius=3)

        # Метричні позначки
        step_m = 0.5
        for mark_m in np.arange(-1.5, 1.6, step_m):
            mx_px = int(self.origin_x + mark_m * self.ppm)
            pygame.draw.line(self.screen, (60, 70, 90), (mx_px, self.rail_y - int(6 * scale)), (mx_px, self.rail_y + int(6 * scale)), max(1, int(2 * scale)))
            lbl = self.font_small.render(f"{mark_m:+.1f}м" if abs(mark_m) > 0.01 else "0.0м", True, (110, 125, 150))
            self.screen.blit(lbl, (mx_px - int(14 * scale), self.rail_y + int(10 * scale)))

    def _render_cart(self) -> None:
        """Відмальовує візок та колеса."""
        scale = self.scene_scale
        x_c_px = int(self.origin_x + self.sim.dynamics.state[0] * self.ppm)
        cart_rect = pygame.Rect(
            x_c_px - self.cart_w_px // 2,
            self.rail_y - self.cart_h_px // 2,
            self.cart_w_px,
            self.cart_h_px
        )

        cart_col = (45, 95, 155) if not self.dragging_cart else (70, 135, 215)
        pygame.draw.rect(self.screen, cart_col, cart_rect, border_radius=int(7 * scale))
        pygame.draw.rect(self.screen, (135, 180, 235), cart_rect, width=max(1, int(2 * scale)), border_radius=int(7 * scale))

        # Лінія перетягування мишкою (пружний ефект)
        if self.dragging_cart:
            mx_px = int(self.origin_x + self.mouse_target_x * self.ppm)
            pygame.draw.line(self.screen, (255, 215, 60), (x_c_px, self.rail_y), (mx_px, self.rail_y), max(2, int(3 * scale)))
            pygame.draw.circle(self.screen, (255, 215, 60), (mx_px, self.rail_y), max(4, int(7 * scale)))

        # Колеса
        wheel_r = max(9, int(13 * scale))
        w_off = self.cart_w_px // 3
        for w_x in [x_c_px - w_off, x_c_px + w_off]:
            pos = (w_x, self.rail_y + self.cart_h_px // 2 - 2)
            pygame.draw.circle(self.screen, (25, 28, 36), pos, wheel_r)
            pygame.draw.circle(self.screen, (155, 165, 180), pos, wheel_r - 4)
            pygame.draw.circle(self.screen, (40, 45, 55), pos, max(2, int(4 * scale)))

    def _render_pendulum(self) -> None:
        """Відмальовує ланки подвійного маятника з пропорційними товщинами."""
        scale = self.scene_scale
        cart_pt, joint_pt, tip_pt = self.sim.dynamics.get_cartesian_points()

        # Екранні координати (Y спрямований вниз)
        p0 = (int(self.origin_x + cart_pt[0] * self.ppm), int(self.rail_y - cart_pt[1] * self.ppm))
        p1 = (int(self.origin_x + joint_pt[0] * self.ppm), int(self.rail_y - joint_pt[1] * self.ppm))
        p2 = (int(self.origin_x + tip_pt[0] * self.ppm), int(self.rail_y - tip_pt[1] * self.ppm))

        # Товщини ланок
        w_l1 = max(5, int(9 * scale))
        w_l2 = max(4, int(7 * scale))

        # Ланка 1: синя
        pygame.draw.line(self.screen, (50, 140, 230), p0, p1, w_l1)
        # Ланка 2: помаранчева
        pygame.draw.line(self.screen, (245, 130, 40), p1, p2, w_l2)

        # Шарнір 1 на візку
        r_p0 = max(6, int(9 * scale))
        pygame.draw.circle(self.screen, (255, 255, 255), p0, r_p0)
        pygame.draw.circle(self.screen, (25, 30, 40), p0, max(2, int(3 * scale)))

        # Проміжний шарнір 2
        r_p1 = max(7, int(10 * scale))
        pygame.draw.circle(self.screen, (255, 255, 255), p1, r_p1)
        pygame.draw.circle(self.screen, (50, 140, 230), p1, max(3, int(5 * scale)))

        # Кінцевий вантаж ланки 2
        r_p2 = max(9, int(14 * scale))
        pygame.draw.circle(self.screen, (245, 130, 40), p2, r_p2)
        pygame.draw.circle(self.screen, (255, 255, 255), p2, r_p2, width=max(1, int(2 * scale)))
        pygame.draw.circle(self.screen, (255, 215, 160), (p2[0] - 2, p2[1] - 2), max(2, int(4 * scale)))

    def _render_force_vector(self) -> None:
        """Малює вектор сили мотора, прикладеної до візка."""
        scale = self.scene_scale
        u_val = self.sim.dynamics.last_u
        if abs(u_val) > 0.4:
            x_c_px = int(self.origin_x + self.sim.dynamics.state[0] * self.ppm)
            y_arr = self.rail_y - self.cart_h_px // 2 - int(14 * scale)
            f_len = int((u_val / self.cfg.F_max) * (85 * scale))

            col = (50, 210, 110) if u_val > 0 else (245, 110, 40)
            pygame.draw.line(self.screen, col, (x_c_px, y_arr), (x_c_px + f_len, y_arr), max(3, int(5 * scale)))

            sign = 1 if f_len > 0 else -1
            tip_x = x_c_px + f_len
            arrow_size = int(10 * scale)
            pygame.draw.polygon(self.screen, col, [
                (tip_x, y_arr),
                (tip_x - sign * arrow_size, y_arr - arrow_size // 2),
                (tip_x - sign * arrow_size, y_arr + arrow_size // 2)
            ])

    def _render_hud(self) -> None:
        """Відображає адаптивні заголовок, статус, підказки та телеметрію."""
        scale = self.ui_scale
        pad_x = int(24 * scale)

        title = self.font_title.render("DOUBLE INVERTED PENDULUM: HYBRID MPC + LQR", True, (240, 245, 255))
        self.screen.blit(title, (pad_x, int(12 * scale)))

        # Статусний бейдж
        mode = self.sim.mode
        telem = self.sim.get_info()

        if mode == ControlMode.LQR_BALANCE:
            col = (50, 210, 110)
        elif mode == ControlMode.LQR_DOWN_ZERO:
            col = (45, 215, 215)
        elif mode == ControlMode.MPC_SWINGUP:
            col = (50, 185, 240)
        elif mode == ControlMode.FALLEN:
            col = (245, 140, 45)
        else:
            col = (150, 155, 170)

        status_text = f"РЕЖИМ: {mode.value}"
        if telem.get("is_zeroed", False) and mode in (ControlMode.LQR_DOWN_ZERO, ControlMode.FALLEN):
            status_text += "  ★ [100% ЗАНУЛЕНО — ГОТОВИЙ ДО SWING-UP]"
            col = (60, 240, 150)

        badge = self.font_badge.render(status_text, True, col)
        self.screen.blit(badge, (pad_x, int(34 * scale)))

        msg = self.font_main.render(self.sim.status_message, True, (205, 220, 240))
        self.screen.blit(msg, (pad_x, int(56 * scale)))

        hints = self.font_small.render(
            "Мишка: тягнути візок | Пробіл: Swing-Up | Z: Занулити | Стрілки ◄ ►: поштовх | [ / ]: Масштаб UI (±) | M: мотор | R: скинути | F11: екран",
            True, (140, 155, 180)
        )
        self.screen.blit(hints, (pad_x, int(76 * scale)))

        # Смуга прогресу MPC під час маневру
        if self.sim.mode == ControlMode.MPC_SWINGUP:
            self.progress_bar.draw(self.screen, self.font_small, self.sim.mpc.get_progress())

        # Осцилограф
        self.oscilloscope.draw(self.screen, self.font_small)

        # Картка телеметрії (компактна)
        card_w = int(280 * scale)
        card_h = int(115 * scale)
        card_x = self.oscilloscope.rect.x - card_w - int(14 * scale)
        card_y = self.oscilloscope.rect.y

        card_rect = pygame.Rect(card_x, card_y, card_w, card_h)
        pygame.draw.rect(self.screen, (22, 27, 38), card_rect, border_radius=6)
        pygame.draw.rect(self.screen, (45, 55, 75), card_rect, width=1, border_radius=6)

        fps = int(self.clock.get_fps())
        lines = [
            f"Кут θ1: {telem['theta1_deg']:+5.1f}° | θ2: {telem['theta2_deg']:+5.1f}°",
            f"Швидкість ω1: {telem['omega1_deg_s']:+5.1f}°/с | ω2: {telem['omega2_deg_s']:+5.1f}°/с",
            f"Позиція x: {telem['x']:+5.2f} м | v: {telem['v']:+5.2f} м/с",
            f"Прискорення a: {telem['a']:+5.1f} м/с² (a_max: {self.cfg.a_max:.0f})",
            f"Сила мотора u: {telem['u']:+5.1f} Н | FPS: {fps}"
        ]

        spacing_y = int(20 * scale)
        for i, line in enumerate(lines):
            t_surf = self.font_small.render(line, True, (215, 225, 245))
            self.screen.blit(t_surf, (card_x + int(10 * scale), card_y + int(8 * scale) + i * spacing_y))

    def _render_controls(self) -> None:
        """Відмальовує компактну нижню панель інтерактивних контролів."""
        scale = self.ui_scale
        pygame.draw.rect(self.screen, (22, 26, 36), self.panel_rect, border_radius=int(8 * scale))
        pygame.draw.rect(self.screen, (45, 54, 75), self.panel_rect, width=1, border_radius=int(8 * scale))

        panel_title = self.font_main.render("КЕРУВАННЯ ТА ПАРАМЕТРИ СТАБІЛІЗАЦІЇ", True, (210, 225, 245))
        self.screen.blit(panel_title, (self.panel_rect.x + int(16 * scale), self.panel_rect.y + int(8 * scale)))

        for sld in self.sliders:
            sld.draw(self.screen, self.font_small)

        for btn in self.buttons:
            btn.draw(self.screen, self.font_button)


def run_app():
    """Точка входу для запуску симулятора."""
    app = HybridDoublePendulumApp()
    app.run()


if __name__ == "__main__":
    run_app()
