"""
main.py
=======
Головний інтерактивний додаток стабілізації одинарного перевернутого маятника
на візку (MPC Swing-Up + LQR) у Pygame 60 FPS.
Повністю синхронізований з фізичним стендом:
- Точні габарити: рейка 58.9 см, візок 4.6 см, стрижень 35 см з вантажем на 30 см.
- Omnipresent LQR: миттєве захоплення балансу при наближенні до вершини.
- Прорахунок та візуалізація розгону MPC перед заливкою в залізо.
"""

import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import pygame

try:
    from .config import DEFAULT_CONFIG, SinglePendulumConfig
    from .dynamics import wrap_to_pi
    from .simulation_manager import ControlMode, SimulationManager
    from .ui import Button, Oscilloscope, ProgressBar, Slider
except ImportError:
    from config import DEFAULT_CONFIG, SinglePendulumConfig
    from dynamics import wrap_to_pi
    from simulation_manager import ControlMode, SimulationManager
    from ui import Button, Oscilloscope, ProgressBar, Slider


class SinglePendulumApp:
    """
    Головний клас графічного інтерфейсу Pygame для одинарного маятника.
    """

    def __init__(self, config: SinglePendulumConfig = DEFAULT_CONFIG):
        self.cfg = config
        pygame.init()
        pygame.display.set_caption(
            "Стабілізація одинарного маятника (MPC Swing-Up + LQR) — Pygame 60 FPS"
        )

        # Визначення роздільної здатності дисплея
        info = pygame.display.Info()
        screen_w, screen_h = info.current_w, info.current_h

        if screen_w >= 2000:
            self.width = min(1800, int(screen_w * 0.80))
            self.height = min(1100, int(screen_h * 0.80))
        elif screen_w >= 1500:
            self.width = min(1400, int(screen_w * 0.82))
            self.height = min(880, int(screen_h * 0.82))
        else:
            self.width = min(1180, max(800, screen_w - 60))
            self.height = min(740, max(580, screen_h - 80))

        self.is_fullscreen = False
        self.screen = pygame.display.set_mode(
            (self.width, self.height), pygame.RESIZABLE
        )
        self.clock = pygame.time.Clock()

        # Менеджер симуляції
        self.sim = SimulationManager(self.cfg)

        # Стан взаємодії
        self.dragging_cart = False
        self.dragging_bob = False
        self.mouse_target_x = 0.0

        # Осцилограф та прогрес-бар
        self.oscilloscope = Oscilloscope(self.width - 320, 20, 300, 130)
        self.progress_bar = ProgressBar(30, 130, 400, 16)

        # Оновлення геометрії
        self._update_layout()

    def _update_layout(self) -> None:
        """Оновлює шрифти, геометрію кнопок і масштаб під поточний розмір вікна."""
        self.scene_scale = max(0.85, min(self.width / 1300.0, self.height / 780.0))

        # Шрифти
        f_size_main = max(13, int(15 * self.scene_scale))
        f_size_title = max(17, int(20 * self.scene_scale))
        f_size_small = max(11, int(12 * self.scene_scale))

        try:
            self.font_main = pygame.font.SysFont(
                "DejaVu Sans,Ubuntu,Arial,Segoe UI", f_size_main
            )
            self.font_title = pygame.font.SysFont(
                "DejaVu Sans,Ubuntu,Arial,Segoe UI", f_size_title, bold=True
            )
            self.font_small = pygame.font.SysFont(
                "DejaVu Sans,Ubuntu,Arial,Segoe UI", f_size_small
            )
            self.font_mono = pygame.font.SysFont(
                "DejaVu Sans Mono,Consolas,Courier New", f_size_main
            )
        except Exception:
            self.font_main = pygame.font.Font(None, f_size_main + 2)
            self.font_title = pygame.font.Font(None, f_size_title + 2)
            self.font_small = pygame.font.Font(None, f_size_small + 2)
            self.font_mono = pygame.font.Font(None, f_size_main + 2)

        # Фізичний центр рейки на екрані
        self.origin_x = self.width // 2
        self.rail_y = int(self.height * 0.48)

        # Масштаб пікселів на метр (PPM):
        # Рейка 58.9 см повинна займати комфортну частину екрана (~60% ширини)
        self.ppm = int((self.width * 0.62) / self.cfg.rail_length_m)

        # Реальні розміри візка у пікселях
        self.cart_w_px = max(26, int(self.cfg.cart_width_m * self.ppm))
        self.cart_h_px = max(18, int(self.cart_w_px * 0.55))
        self.rod_len_px = int(self.cfg.L * self.ppm)
        self.bob_com_px = int(self.cfg.l_com * self.ppm)

        # Нижній пульт керування (панель кнопок і повзунків)
        panel_y = self.height - int(195 * self.scene_scale)
        slider_w = int(210 * self.scene_scale)
        slider_h = 10
        gap = int(24 * self.scene_scale)

        col1_x = 30
        col2_x = col1_x + slider_w + gap

        # Отримання базових коефіцієнтів
        k_x, k_th, k_v, k_om = self.sim.lqr.K

        v_kth = (
            getattr(self, "slider_kth", None).val
            if hasattr(self, "slider_kth")
            else k_th
        )
        v_kom = (
            getattr(self, "slider_kom", None).val
            if hasattr(self, "slider_kom")
            else k_om
        )
        v_kx = (
            getattr(self, "slider_kx", None).val if hasattr(self, "slider_kx") else k_x
        )
        v_kv = (
            getattr(self, "slider_kv", None).val if hasattr(self, "slider_kv") else k_v
        )

        self.slider_kth = Slider(
            col1_x,
            panel_y + 40,
            slider_w,
            slider_h,
            20.0,
            300.0,
            v_kth,
            "k_θ (кутова жорсткість)",
            color=(230, 80, 60),
        )
        self.slider_kom = Slider(
            col1_x,
            panel_y + 95,
            slider_w,
            slider_h,
            2.0,
            60.0,
            v_kom,
            "k_ω (кутовий демпфер)",
            color=(235, 130, 45),
        )
        self.slider_kx = Slider(
            col2_x,
            panel_y + 40,
            slider_w,
            slider_h,
            1.0,
            60.0,
            v_kx,
            "k_x (центрування рейки)",
            color=(50, 150, 235),
        )
        self.slider_kv = Slider(
            col2_x,
            panel_y + 95,
            slider_w,
            slider_h,
            1.0,
            40.0,
            v_kv,
            "k_v (демпфер візка)",
            color=(60, 190, 160),
        )

        self.sliders = [
            self.slider_kth,
            self.slider_kom,
            self.slider_kx,
            self.slider_kv,
        ]

        # Кнопки дій
        btn_w = int(145 * self.scene_scale)
        btn_h = int(36 * self.scene_scale)
        btn_col1 = col2_x + slider_w + gap + 10
        btn_col2 = btn_col1 + btn_w + 12

        self.btn_mpc = Button(
            btn_col1,
            panel_y + 25,
            btn_w,
            btn_h,
            "▶ MPC Swing-Up [S]",
            (45, 120, 210),
            (65, 145, 240),
            self.sim.trigger_mpc_swingup,
        )
        self.btn_zero = Button(
            btn_col1,
            panel_y + 75,
            btn_w,
            btn_h,
            "★ Занулити (Down) [D]",
            (35, 125, 95),
            (50, 155, 115),
            self.sim.trigger_down_zero,
        )

        motor_txt = (
            "Мотор: УВІМК [Space]"
            if self.sim.mode != ControlMode.MANUAL_OFF
            else "Мотор: ВИМК [Space]"
        )
        self.btn_motor = Button(
            btn_col2,
            panel_y + 25,
            btn_w,
            btn_h,
            motor_txt,
            (40, 130, 85) if self.sim.mode != ControlMode.MANUAL_OFF else (150, 45, 45),
            (55, 165, 110)
            if self.sim.mode != ControlMode.MANUAL_OFF
            else (190, 60, 60),
            self.sim.toggle_motor,
        )
        self.btn_reset = Button(
            btn_col2,
            panel_y + 75,
            btn_w,
            btn_h,
            "↻ Скинути [R]",
            (85, 95, 115),
            (110, 120, 140),
            self.sim.reset,
        )

        self.buttons = [self.btn_mpc, self.btn_zero, self.btn_motor, self.btn_reset]

        # Оновлення позиції осцилографа
        self.oscilloscope.rect = pygame.Rect(self.width - 320, 20, 300, 130)

    def toggle_fullscreen(self) -> None:
        self.is_fullscreen = not self.is_fullscreen
        if self.is_fullscreen:
            info = pygame.display.Info()
            self.width, self.height = info.current_w, info.current_h
            self.screen = pygame.display.set_mode(
                (self.width, self.height), pygame.FULLSCREEN
            )
        else:
            self.width, self.height = 1240, 760
            self.screen = pygame.display.set_mode(
                (self.width, self.height), pygame.RESIZABLE
            )
        self._update_layout()

    def run(self) -> None:
        running = True
        while running:
            dt_raw = self.clock.tick(self.cfg.fps) / 1000.0
            dt = min(0.04, max(0.001, dt_raw))

            # Передача ручних коефіцієнтів LQR якщо змінено
            custom_g = (
                self.slider_kth.val,
                self.slider_kom.val,
                self.slider_kx.val,
                self.slider_kv.val,
            )
            self.sim.lqr.K = np.array(
                [custom_g[2], custom_g[0], custom_g[3], custom_g[1]]
            )

            # Обробка подій
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.width, self.height = event.w, event.h
                    self.screen = pygame.display.set_mode(
                        (self.width, self.height), pygame.RESIZABLE
                    )
                    self._update_layout()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key in (pygame.K_F11, pygame.K_f):
                        self.toggle_fullscreen()
                    elif event.key in (pygame.K_s, pygame.K_RETURN):
                        self.sim.trigger_mpc_swingup()
                    elif event.key == pygame.K_d:
                        self.sim.trigger_down_zero()
                    elif event.key == pygame.K_SPACE:
                        self.sim.toggle_motor()
                        self.btn_motor.text = (
                            "Мотор: УВІМК [Space]"
                            if self.sim.mode != ControlMode.MANUAL_OFF
                            else "Мотор: ВИМК [Space]"
                        )
                        self.btn_motor.color = (
                            (40, 130, 85)
                            if self.sim.mode != ControlMode.MANUAL_OFF
                            else (150, 45, 45)
                        )
                    elif event.key == pygame.K_r:
                        self.sim.reset()
                    elif event.key == pygame.K_LEFT:
                        self.sim.apply_kick(-1.0)
                    elif event.key == pygame.K_RIGHT:
                        self.sim.apply_kick(1.0)

                # Події UI
                for btn in self.buttons:
                    btn.handle_event(event)
                for sld in self.sliders:
                    sld.handle_event(event)

                # Взаємодія мишкою з візком
                x_c_px = int(self.origin_x + self.sim.dynamics.state[0] * self.ppm)
                cart_rect = pygame.Rect(
                    x_c_px - self.cart_w_px // 2,
                    self.rail_y - self.cart_h_px // 2,
                    self.cart_w_px,
                    self.cart_h_px,
                )

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if cart_rect.collidepoint(event.pos):
                        self.dragging_cart = True
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    if self.dragging_cart:
                        self.dragging_cart = False
                        self.sim.release_mouse_drag()
                elif event.type == pygame.MOUSEMOTION and self.dragging_cart:
                    target_m = (event.pos[0] - self.origin_x) / float(self.ppm)
                    self.sim.set_mouse_drag_target(target_m)

            # Оновлення фізики (кілька субстепів для стабільності RK4)
            substeps = 4
            sub_dt = dt / float(substeps)
            for _ in range(substeps):
                self.sim.update(sub_dt)

            # Оновлення осцилографа
            th_deg = wrap_to_pi(self.sim.dynamics.state[1]) * 180.0 / math.pi
            cart_cm = self.sim.dynamics.state[0] * 100.0
            self.oscilloscope.push(th_deg, cart_cm)

            # Рендеринг сцени
            self._render()
            pygame.display.flip()

        pygame.quit()

    def _render(self) -> None:
        self.screen.fill((16, 20, 28))

        # 1. Заголовок і статус
        title_surf = self.font_title.render(
            "СТАБІЛІЗАЦІЯ ОДИНАРНОГО ПЕРЕВЕРНУТОГО МАЯТНИКА", True, (240, 245, 255)
        )
        self.screen.blit(title_surf, (30, 20))

        mode_col = (
            (70, 220, 130)
            if self.sim.mode == ControlMode.LQR_BALANCE
            else (190, 80, 230)
            if self.sim.mode == ControlMode.MPC_SWINGUP
            else (70, 170, 240)
            if self.sim.mode == ControlMode.LQR_DOWN_ZERO
            else (240, 190, 60)
            if self.sim.mode == ControlMode.WAITING_FOR_TRIGGER
            else (200, 70, 70)
        )
        mode_surf = self.font_main.render(
            f"Режим: {self.sim.mode.value}", True, mode_col
        )
        self.screen.blit(mode_surf, (30, 52))

        stat_surf = self.font_small.render(
            self.sim.status_message, True, (170, 185, 205)
        )
        self.screen.blit(stat_surf, (30, 80))

        # 2. Індикатор прогресу MPC
        if (
            self.sim.mode == ControlMode.MPC_SWINGUP
            and self.sim.mpc.loaded
            and self.sim.mpc.duration > 0
        ):
            prog = self.sim.mpc.playback_time / self.sim.mpc.duration
            self.progress_bar.draw(self.screen, prog, self.font_small, "Прогрес MPC")

        # 3. Рейка стенду
        x_lim = self.cfg.x_limit_physical  # 0.2945 м
        rail_start_px = int(self.origin_x - x_lim * self.ppm)
        rail_end_px = int(self.origin_x + x_lim * self.ppm)

        # Металева рейка
        pygame.draw.line(
            self.screen,
            (40, 48, 65),
            (rail_start_px - 20, self.rail_y),
            (rail_end_px + 20, self.rail_y),
            10,
        )
        pygame.draw.line(
            self.screen,
            (95, 110, 140),
            (rail_start_px - 20, self.rail_y - 2),
            (rail_end_px + 20, self.rail_y - 2),
            2,
        )

        # Фізичні кінцеві стопери (червоні)
        pygame.draw.rect(
            self.screen,
            (220, 50, 60),
            (rail_start_px - 14, self.rail_y - 24, 14, 48),
            border_radius=3,
        )
        pygame.draw.rect(
            self.screen,
            (220, 50, 60),
            (rail_end_px, self.rail_y - 24, 14, 48),
            border_radius=3,
        )

        # Безпечна зона Swing-Up (±19 см)
        safe_x = self.cfg.x_rail
        safe_l_px = int(self.origin_x - safe_x * self.ppm)
        safe_r_px = int(self.origin_x + safe_x * self.ppm)
        pygame.draw.line(
            self.screen,
            (200, 160, 40),
            (safe_l_px, self.rail_y - 12),
            (safe_l_px, self.rail_y + 12),
            2,
        )
        pygame.draw.line(
            self.screen,
            (200, 160, 40),
            (safe_r_px, self.rail_y - 12),
            (safe_r_px, self.rail_y + 12),
            2,
        )

        # Мітка центру 0.00 м
        pygame.draw.line(
            self.screen,
            (60, 75, 100),
            (self.origin_x, self.rail_y - 8),
            (self.origin_x, self.rail_y + 8),
            2,
        )
        lbl_center = self.font_small.render("0.0 см (Центр)", True, (110, 125, 150))
        self.screen.blit(
            lbl_center, (self.origin_x - lbl_center.get_width() // 2, self.rail_y + 16)
        )

        # 4. Візок (вагонетка) з реальними пропорціями (4.6 см)
        cart_x = self.sim.dynamics.state[0]
        cart_v = self.sim.dynamics.state[2]
        x_c_px = int(self.origin_x + cart_x * self.ppm)

        cart_rect = pygame.Rect(
            x_c_px - self.cart_w_px // 2,
            self.rail_y - self.cart_h_px // 2,
            self.cart_w_px,
            self.cart_h_px,
        )
        cart_col = (70, 130, 210) if self.dragging_cart else (45, 95, 160)
        pygame.draw.rect(self.screen, cart_col, cart_rect, border_radius=4)
        pygame.draw.rect(
            self.screen, (140, 185, 240), cart_rect, width=2, border_radius=4
        )

        # Колеса візка
        wheel_r = max(4, int(self.cart_h_px * 0.35))
        w_off = int(self.cart_w_px * 0.28)
        for wx in [x_c_px - w_off, x_c_px + w_off]:
            pygame.draw.circle(
                self.screen,
                (25, 30, 40),
                (wx, self.rail_y + self.cart_h_px // 2),
                wheel_r,
            )
            pygame.draw.circle(
                self.screen,
                (130, 145, 165),
                (wx, self.rail_y + self.cart_h_px // 2),
                wheel_r - 2,
            )

        # 5. Маятник (стрижень 35 см з круглим вантажем COM на 30 см)
        th = self.sim.dynamics.state[1]
        w_speed = self.sim.dynamics.state[3]

        # Вісь шарніра на візку
        pivot_px = (x_c_px, self.rail_y)

        # Координати кінця стрижня та центру мас вантажу
        end_x = int(x_c_px + self.rod_len_px * math.sin(th))
        end_y = int(self.rail_y - self.rod_len_px * math.cos(th))
        com_x = int(x_c_px + self.bob_com_px * math.sin(th))
        com_y = int(self.rail_y - self.bob_com_px * math.cos(th))

        # Стрижень (алюмінієвий стержень)
        pygame.draw.line(self.screen, (200, 215, 235), pivot_px, (end_x, end_y), 4)

        # Вантаж маятника (важкий циліндричний вантаж 106 г)
        bob_r = max(9, int(13 * self.scene_scale))
        pygame.draw.circle(self.screen, (230, 140, 40), (com_x, com_y), bob_r)
        pygame.draw.circle(self.screen, (255, 190, 100), (com_x, com_y), bob_r - 2)
        pygame.draw.circle(self.screen, (40, 45, 55), (com_x, com_y), 3)

        # Підшипник шарніра (енкодер)
        pygame.draw.circle(self.screen, (30, 35, 45), pivot_px, 6)
        pygame.draw.circle(
            self.screen,
            (70, 220, 130)
            if abs(wrap_to_pi(th)) < np.radians(self.cfg.catch_angle_deg)
            else (200, 210, 225),
            pivot_px,
            4,
        )

        # 6. Телеметрія праворуч
        deg_th = wrap_to_pi(th) * 180.0 / math.pi
        telemetry_lines = [
            f"X: {cart_x * 100.0:+6.2f} см",
            f"V: {cart_v:+6.2f} м/с",
            f"θ: {deg_th:+6.1f}°",
            f"ω: {w_speed:+6.2f} рад/с",
            f"Сила мотора: {self.sim.dynamics.last_u:+5.2f} Н",
            f"Прискорення: {self.sim.dynamics.last_a:+5.2f} м/с²",
        ]
        info_x = self.width - 290
        info_y = 170
        pygame.draw.rect(
            self.screen,
            (20, 26, 38),
            (info_x - 12, info_y - 8, 280, len(telemetry_lines) * 22 + 14),
            border_radius=6,
        )
        pygame.draw.rect(
            self.screen,
            (45, 55, 75),
            (info_x - 12, info_y - 8, 280, len(telemetry_lines) * 22 + 14),
            width=1,
            border_radius=6,
        )

        for i, line in enumerate(telemetry_lines):
            t_surf = self.font_mono.render(line, True, (215, 230, 250))
            self.screen.blit(t_surf, (info_x, info_y + i * 22))

        # 7. Осцилограф
        self.oscilloscope.draw(self.screen, self.font_small)

        # 8. Кнопки та повзунки
        for btn in self.buttons:
            btn.draw(self.screen, self.font_main)
        for sld in self.sliders:
            sld.draw(self.screen, self.font_small, self.font_mono)


if __name__ == "__main__":
    app = SinglePendulumApp()
    app.run()
