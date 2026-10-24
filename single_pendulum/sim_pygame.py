"""
=============================================================================
РЕАЛЬНОЧАСОВИЙ (REALTIME 60 FPS) СИМУЛЯТОР СТАБІЛІЗАЦІЇ ТА ПІДЙОМУ (SWING-UP)
ОДИНАРНОГО ПЕРЕВЕРНУТОГО МАЯТНИКА НА ВІЗКУ (SINGLE INVERTED PENDULUM)
=============================================================================
Можливості:
1. MPC SWING-UP (ПОПЕРЕДНЬО РОЗРАХОВАНА ТРАЄКТОРІЯ):
   Оптимальний підйом з урахуванням суворих лімітів швидкості (v_max = 0.66 м/с)
   та прискорення (a_max = 15.0 м/с²), згенерований через CasADi/IPOPT.
2. ЕНЕРГЕТИЧНИЙ SWING-UP (Åström / Spong):
   Неперервне резонансне розгойдування за повною механічною енергією.
3. АВТОМАТИЧНИЙ ПЕРЕХІД ДО LQR:
   Щойно маятник наближається до вертикалі (|theta| < 25°), LQR-регулятор
   перехоплює керування та стабілізує систему у верхній точці.
4. ПОВНИЙ ЕКРАН / МАСШТАБУВАННЯ (F11 або кнопка '⛶ Екран').
5. Інтерактивні слайдери для LQR-коефіцієнтів та меж a_max, v_max.
6. Осцилограф кута в реальному часі та фізична взаємодія мишкою.
=============================================================================
"""

import sys
import math
from pathlib import Path
import numpy as np
import pygame

# Додаємо поточну теку до sys.path для коректного імпорту модулів
sys_path_dir = str(Path(__file__).resolve().parent)
if sys_path_dir not in sys.path:
    sys.path.insert(0, sys_path_dir)

from config import SinglePendulumConfig, DEFAULT_CONFIG
from lqr_controller import SinglePendulumLQR
from mpc_controller import SinglePendulumMPCController


# =============================================================================
# 1. ЕЛЕМЕНТИ ІНТЕРФЕЙСУ (СЛАЙДЕРИ ТА КНОПКИ)
# =============================================================================
class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, init_val, label, unit="", color=(60, 130, 220)):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = init_val
        self.label = label
        self.unit = unit
        self.color = color
        self.dragging = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            hitbox = self.rect.inflate(12, 14)
            if hitbox.collidepoint(event.pos):
                self.dragging = True
                self._update_val(event.pos[0])
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self._update_val(event.pos[0])
            return True
        return False

    def _update_val(self, mouse_x):
        rel = (mouse_x - self.rect.x) / self.rect.w
        rel = max(0.0, min(1.0, rel))
        self.val = self.min_val + rel * (self.max_val - self.min_val)

    def draw(self, surface, font):
        pygame.draw.rect(surface, (40, 45, 60), self.rect, border_radius=4)
        rel = (self.val - self.min_val) / max(1e-6, self.max_val - self.min_val)
        fill_w = max(4, int(rel * self.rect.w))
        fill_rect = pygame.Rect(self.rect.x, self.rect.y, fill_w, self.rect.h)
        pygame.draw.rect(surface, self.color, fill_rect, border_radius=4)

        handle_x = self.rect.x + fill_w
        handle_rect = pygame.Rect(handle_x - 5, self.rect.y - 4, 10, self.rect.h + 8)
        handle_color = (255, 255, 255) if self.dragging else (210, 225, 245)
        pygame.draw.rect(surface, handle_color, handle_rect, border_radius=3)
        pygame.draw.rect(surface, (20, 25, 35), handle_rect, width=1, border_radius=3)

        val_text = f"{self.val:.1f}{self.unit}"
        txt = font.render(f"{self.label}: {val_text}", True, (215, 225, 240))
        surface.blit(txt, (self.rect.x, self.rect.y - 18))


class Button:
    def __init__(self, x, y, w, h, text, color, hover_color, callback):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.callback = callback
        self.hovered = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.callback()
                return True
        return False

    def draw(self, surface, font):
        bg = self.hover_color if self.hovered else self.color
        pygame.draw.rect(surface, bg, self.rect, border_radius=6)
        pygame.draw.rect(surface, (255, 255, 255, 45), self.rect, width=1, border_radius=6)
        txt = font.render(self.text, True, (240, 245, 255))
        txt_rect = txt.get_rect(center=self.rect.center)
        surface.blit(txt, txt_rect)


# =============================================================================
# 2. ГОЛОВНИЙ КЛАС СИМУЛЯТОРА
# =============================================================================
class RealtimeCartPoleSim:
    def __init__(self, cfg: SinglePendulumConfig = DEFAULT_CONFIG):
        pygame.init()
        pygame.display.set_caption("Одинарний перевернутий маятник (MPC Swing-Up + LQR) — Pygame 60 FPS")

        self.cfg = cfg
        self.lqr = SinglePendulumLQR(cfg)
        self.mpc = SinglePendulumMPCController(cfg)

        self.width = 1200
        self.height = 760
        self.is_fullscreen = False

        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        self.font_title = pygame.font.SysFont("dejavusans", 16, bold=True)
        self.font_main  = pygame.font.SysFont("dejavusans", 13)
        self.font_small = pygame.font.SysFont("dejavusans", 11)

        # Стан системи: [x, theta, v, omega]
        # theta = 0 — вертикаль вгору, theta = pi — вертикаль вниз
        self.state = np.array([0.0, np.radians(self.cfg.init_theta_deg), 0.0, 0.0], dtype=float)
        self.motor_enabled = True
        self.last_u = 0.0
        self.last_a = 0.0
        self.current_mode = "LQR"
        self.swing_kick_dir = 1.0

        # Миша: захоплення
        self.dragging_bob = False
        self.dragging_cart = False
        self.mouse_target_x = 0.0
        self.mouse_target_th = 0.0

        # Історія для осцилографа
        self.theta_history = []
        self.max_hist_len = 220

        # Оновлення динамічного розташування UI
        self._init_ui()

    def _init_ui(self):
        # Базові оптимальні коефіцієнти LQR
        k_top = self.lqr.K_top
        # Конвенція знаків: u = k_th * th + k_om * w + k_x * x + k_v * v
        # де K_top[1]=th, K_top[3]=omega, K_top[0]=x, K_top[2]=v
        self.opt_kth = abs(k_top[1])
        self.opt_kom = abs(k_top[3])
        self.opt_kx  = abs(k_top[0])
        self.opt_kv  = abs(k_top[2])

        self._update_layout()

    def _update_layout(self):
        self.origin_x = self.width // 2
        self.rail_y = int(self.height * 0.44)

        # Довжина рейки займає ~70% ширини екрана
        available_w = max(600, self.width - 240)
        self.ppm = available_w / (2.0 * (self.cfg.x_rail + 0.25))

        self.cart_w_px = max(65, int(0.32 * self.ppm))
        self.cart_h_px = max(24, int(0.14 * self.ppm))
        self.rod_len_px = int(self.cfg.L * self.ppm)

        # Нижня панель UI
        panel_y = max(490, self.height - 200)
        slider_w = min(190, max(130, int(self.width * 0.15)))
        slider_h = 10
        gap = min(30, max(12, int(self.width * 0.02)))

        col1_x = 40
        col2_x = col1_x + slider_w + gap
        col3_x = col2_x + slider_w + gap

        v_kth  = getattr(self, 'slider_kth', None).val if hasattr(self, 'slider_kth') else self.opt_kth
        v_kom  = getattr(self, 'slider_kom', None).val if hasattr(self, 'slider_kom') else self.opt_kom
        v_kx   = getattr(self, 'slider_kx', None).val if hasattr(self, 'slider_kx') else self.opt_kx
        v_kv   = getattr(self, 'slider_kv', None).val if hasattr(self, 'slider_kv') else self.opt_kv
        v_amax = getattr(self, 'slider_amax', None).val if hasattr(self, 'slider_amax') else self.cfg.a_max
        v_vmax = getattr(self, 'slider_vmax', None).val if hasattr(self, 'slider_vmax') else self.cfg.v_max

        self.slider_kth  = Slider(col1_x, panel_y + 55,  slider_w, slider_h, 0.0, 300.0, v_kth,  "k_θ (кут)", color=(230, 80, 60))
        self.slider_kom  = Slider(col1_x, panel_y + 115, slider_w, slider_h, 0.0, 80.0,  v_kom,  "k_ω (демпфер)", color=(235, 130, 45))
        self.slider_kx   = Slider(col2_x, panel_y + 55,  slider_w, slider_h, 0.0, 60.0,  v_kx,   "k_x (рейка)", color=(50, 150, 235))
        self.slider_kv   = Slider(col2_x, panel_y + 115, slider_w, slider_h, 0.0, 50.0,  v_kv,   "k_v (швидкість)", color=(60, 190, 160))
        self.slider_amax = Slider(col3_x, panel_y + 55,  slider_w, slider_h, 2.0, 35.0,  v_amax, "a_max (прискор.)", unit=" м/с²", color=(185, 80, 220))
        self.slider_vmax = Slider(col3_x, panel_y + 115, slider_w, slider_h, 0.2, 3.5,   v_vmax, "v_max (макс v)", unit=" м/с", color=(215, 165, 45))

        self.sliders = [self.slider_kth, self.slider_kom, self.slider_kx, self.slider_kv, self.slider_amax, self.slider_vmax]

        # Кнопки дій (2 стовпчики по 3 кнопки)
        btn_w = min(145, max(115, int(self.width * 0.115)))
        btn_h = 36
        btn_col1 = col3_x + slider_w + gap + 10
        btn_col2 = btn_col1 + btn_w + 10

        self.btn_mpc   = Button(btn_col1, panel_y + 35,  btn_w, btn_h, "▶ MPC Swing-Up", (45, 120, 210), (65, 145, 240), self.trigger_mpc_swingup)
        self.btn_lqr   = Button(btn_col1, panel_y + 80,  btn_w, btn_h, "★ Авто LQR",     (35, 95, 165),  (50, 125, 210), self.set_lqr)
        self.btn_motor = Button(btn_col1, panel_y + 125, btn_w, btn_h, "Мотор: УВІМК" if self.motor_enabled else "Мотор: ВИМК",
                                (40, 130, 85) if self.motor_enabled else (150, 45, 45),
                                (55, 165, 110) if self.motor_enabled else (190, 60, 60), self.toggle_motor)

        self.btn_energy= Button(btn_col2, panel_y + 35,  btn_w, btn_h, "⚡ Енерго Swing", (180, 100, 30), (215, 125, 40), self.trigger_energy_swingup)
        self.btn_drop  = Button(btn_col2, panel_y + 80,  btn_w, btn_h, "⬇ Опустити вниз", (160, 75, 40),  (195, 95, 50),  self.drop_down)
        self.btn_reset = Button(btn_col2, panel_y + 125, btn_w, btn_h, "↻ Скинути",      (105, 110, 125),(130, 135, 150),self.reset_state)

        # Кнопка повного екрана
        self.btn_fullscreen = Button(self.width - 125, 16, 95, 30, "⛶ Екран", (45, 55, 75), (65, 80, 105), self.toggle_fullscreen)

        self.buttons = [self.btn_mpc, self.btn_lqr, self.btn_motor, self.btn_energy, self.btn_drop, self.btn_reset, self.btn_fullscreen]

    def toggle_fullscreen(self):
        self.is_fullscreen = not self.is_fullscreen
        if self.is_fullscreen:
            info = pygame.display.Info()
            self.width, self.height = info.current_w, info.current_h
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
        else:
            self.width, self.height = 1200, 760
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self._update_layout()

    def set_lqr(self):
        """Скидає слайдери до теоретично оптимальних коефіцієнтів LQR."""
        self.slider_kth.val = self.opt_kth
        self.slider_kom.val = self.opt_kom
        self.slider_kx.val  = self.opt_kx
        self.slider_kv.val  = self.opt_kv
        self.slider_amax.val = self.cfg.a_max
        self.slider_vmax.val = self.cfg.v_max

    def toggle_motor(self):
        self.motor_enabled = not self.motor_enabled
        if self.motor_enabled:
            self.btn_motor.text = "Мотор: УВІМК"
            self.btn_motor.color = (40, 130, 85)
            self.btn_motor.hover_color = (55, 165, 110)
        else:
            self.mpc.stop()
            self.btn_motor.text = "Мотор: ВИМК"
            self.btn_motor.color = (150, 45, 45)
            self.btn_motor.hover_color = (190, 60, 60)

    def reset_state(self):
        self.mpc.stop()
        self.state = np.array([0.0, np.radians(self.cfg.init_theta_deg), 0.0, 0.0], dtype=float)
        self.current_mode = "LQR"

    def drop_down(self):
        """Опускає маятник у нижнє висяче положення спокою (theta = pi)."""
        self.mpc.stop()
        self.state = np.array([0.0, np.pi, 0.0, 0.0], dtype=float)
        self.swing_kick_dir = 1.0
        self.current_mode = "WAIT"

    def trigger_mpc_swingup(self):
        """Запускає виконання записаної оптимальної траєкторії MPC."""
        if not self.mpc.loaded:
            print("[Помилка] Траєкторія MPC не знайдена. Спочатку згенеруйте її!")
            return
        self.motor_enabled = True
        self.btn_motor.text = "Мотор: УВІМК"
        self.btn_motor.color = (40, 130, 85)
        # Якщо маятник біля верхньої точки, спершу скидаємо вниз
        th_err = (self.state[1] + np.pi) % (2.0 * np.pi) - np.pi
        if abs(th_err) < np.radians(45):
            self.state = np.array([0.0, np.pi, 0.0, 0.0], dtype=float)
        self.mpc.start()
        self.current_mode = "MPC_SWINGUP"

    def trigger_energy_swingup(self):
        """Перемикає на неперервний енергетичний підйом (Åström/Spong)."""
        self.mpc.stop()
        self.motor_enabled = True
        self.btn_motor.text = "Мотор: УВІМК"
        self.btn_motor.color = (40, 130, 85)
        self.current_mode = "ENERGY_SWINGUP"

    # -------------------------------------------------------------------------
    # Розрахунок сили мотора візка
    # -------------------------------------------------------------------------
    def compute_control_force(self, s, dt):
        if not self.motor_enabled:
            return 0.0, "OFF"

        x_val, th_val, v_val, w_val = s
        th_err = (th_val + np.pi) % (2.0 * np.pi) - np.pi

        a_max = self.slider_amax.val
        v_max = self.slider_vmax.val
        u_max = (self.cfg.M + self.cfg.m) * a_max

        # 1. ЗОНА ЗАХОПЛЕННЯ LQR (біля верхньої вертикалі)
        catch_rad = np.radians(self.cfg.catch_angle_deg)
        if abs(th_err) < catch_rad and abs(w_val) < self.cfg.catch_max_omega:
            if self.mpc.is_active:
                self.mpc.stop()
            gains = (self.slider_kth.val, self.slider_kom.val, self.slider_kx.val, self.slider_kv.val)
            u = self.lqr.compute_force(s, target_x=0.0, custom_gains=gains)
            mode = "LQR"

        # 2. РЕЖИМ MPC SWING-UP
        elif self.mpc.is_active:
            u_mpc, should_handover, is_finished = self.mpc.step(dt, s)
            if should_handover:
                self.mpc.stop()
                gains = (self.slider_kth.val, self.slider_kom.val, self.slider_kx.val, self.slider_kv.val)
                u = self.lqr.compute_force(s, target_x=0.0, custom_gains=gains)
                mode = "LQR"
            else:
                u = u_mpc
                mode = "MPC_SWINGUP"

        # 3. РЕЖИМ ЕНЕРГЕТИЧНОГО SWING-UP (Åström / Spong)
        elif self.current_mode == "ENERGY_SWINGUP" or abs(th_err) >= catch_rad:
            # Повна енергія відносно верхньої точки (E = 0 на вершині, E < 0 внизу)
            E = 0.5 * self.cfg.I * (w_val ** 2) + self.cfg.m * self.cfg.g * self.cfg.l * (np.cos(th_err) - 1.0)

            # Стартовий імпульс з мертвої точки
            if abs(th_err) > 2.5 and abs(w_val) < 0.3:
                u_pump = 12.0 * self.swing_kick_dir
            else:
                if abs(x_val) > 0.25:
                    self.swing_kick_dir = -np.sign(x_val)
                # Відкалібрований коефіцієнт для L = 0.35 м
                k_e = 30.0
                u_pump = k_e * E * w_val * np.cos(th_err)

            # М'яке центрування
            u_center = -4.0 * x_val - 2.5 * v_val
            u = u_pump + u_center
            mode = "ENERGY_SWINGUP"
        else:
            u = 0.0
            mode = "WAIT"

        # Фізичне обмеження тяги мотора
        u = float(np.clip(u, -u_max, u_max))

        # Обмеження за максимальною лінійною швидкістю візка (проти-ЕРС приводу)
        if v_val >= v_max and u > 0:
            u = 0.0
        elif v_val <= -v_max and u < 0:
            u = 0.0

        return u, mode

    # -------------------------------------------------------------------------
    # Нелінійна фізична динаміка візок + маятник
    # -------------------------------------------------------------------------
    def dynamics(self, s, external_cart_force=0.0, dt=0.005):
        x_val, th_val, v_val, w_val = s
        u, mode = self.compute_control_force(s, dt)
        self.last_u = u
        self.current_mode = mode

        total_force = u + external_cart_force

        M_val = self.cfg.M
        m_val = self.cfg.m
        l_val = self.cfg.l
        g_val = self.cfg.g
        b_c   = self.cfg.b_cart
        b_p   = self.cfg.b_pend
        a_max = self.slider_amax.val

        sin_t = np.sin(th_val)
        cos_t = np.cos(th_val)

        denom = l_val * (4.0 / 3.0 - m_val * (cos_t ** 2) / (M_val + m_val))
        temp = (total_force - b_c * v_val + m_val * l_val * (w_val ** 2) * sin_t) / (M_val + m_val)

        theta_acc = (g_val * sin_t - cos_t * temp - (b_p / (m_val * l_val)) * w_val) / denom
        x_acc = temp - (m_val * l_val * theta_acc * cos_t) / (M_val + m_val)

        # Фізичне обмеження прискорення осі візка
        if abs(x_acc) > a_max:
            x_acc = float(np.clip(x_acc, -a_max, a_max))
            theta_acc = (g_val * sin_t - x_acc * cos_t - (b_p / (m_val * l_val)) * w_val) / (l_val * (4.0 / 3.0))

        self.last_a = x_acc

        # Пружні гумові відбійники на кінцях рейки
        x_rail = self.cfg.x_rail
        if x_val >= x_rail:
            f_bump = -self.cfg.k_bumper * (x_val - x_rail) - self.cfg.d_bumper * max(0.0, v_val)
            x_acc += f_bump / M_val
        elif x_val <= -x_rail:
            f_bump = -self.cfg.k_bumper * (x_val + x_rail) - self.cfg.d_bumper * min(0.0, v_val)
            x_acc += f_bump / M_val

        return np.array([v_val, w_val, x_acc, theta_acc])

    def rk4_step(self, dt, ext_force=0.0):
        k1 = self.dynamics(self.state, ext_force, dt)
        k2 = self.dynamics(self.state + 0.5 * dt * k1, ext_force, dt)
        k3 = self.dynamics(self.state + 0.5 * dt * k2, ext_force, dt)
        k4 = self.dynamics(self.state + dt * k3, ext_force, dt)
        self.state += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    # -------------------------------------------------------------------------
    # Головний цикл симуляції
    # -------------------------------------------------------------------------
    def run(self):
        running = True
        while running:
            raw_dt = self.clock.tick(self.cfg.fps) / 1000.0
            dt = min(0.04, max(0.001, raw_dt))

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
                        self.toggle_motor()
                    elif event.key == pygame.K_m:
                        self.trigger_mpc_swingup()
                    elif event.key == pygame.K_e:
                        self.trigger_energy_swingup()
                    elif event.key == pygame.K_r:
                        self.reset_state()
                    elif event.key == pygame.K_a:
                        self.set_lqr()
                    elif event.key == pygame.K_d:
                        self.drop_down()
                    elif event.key == pygame.K_LEFT:
                        self.state[2] -= 0.5
                    elif event.key == pygame.K_RIGHT:
                        self.state[2] += 0.5

                ui_captured = False
                for slider in self.sliders:
                    if slider.handle_event(event):
                        ui_captured = True
                for button in self.buttons:
                    if button.handle_event(event):
                        ui_captured = True

                if not ui_captured:
                    self._handle_mouse_physics(event)

            # Сила від перетягування візка мишкою
            ext_cart_force = 0.0
            if self.dragging_cart:
                k_drag = 700.0
                d_drag = 65.0
                cart_x = self.state[0]
                cart_v = self.state[2]
                ext_cart_force = k_drag * (self.mouse_target_x - cart_x) - d_drag * cart_v

            # Фізична інтеграція фіксованими підкроками (стабільність 200 Гц)
            if not self.dragging_bob:
                sub_steps = 4
                dt_sub = dt / sub_steps
                for _ in range(sub_steps):
                    self.rk4_step(dt_sub, ext_cart_force)
            else:
                th_diff = (self.mouse_target_th - self.state[1] + np.pi) % (2.0 * np.pi) - np.pi
                self.state[1] += th_diff * 0.35
                self.state[3] = th_diff / dt

            # Історія кута для осцилографа
            th_deg = np.degrees((self.state[1] + np.pi) % (2.0 * np.pi) - np.pi)
            self.theta_history.append(th_deg)
            if len(self.theta_history) > self.max_hist_len:
                self.theta_history.pop(0)

            self._render()
            pygame.display.flip()

        pygame.quit()
        sys.exit()

    def _handle_mouse_physics(self, event):
        x_c_px = int(self.origin_x + self.state[0] * self.ppm)
        th = self.state[1]
        x_bob_px = int(x_c_px + self.rod_len_px * np.sin(th))
        y_bob_px = int(self.rail_y - self.rod_len_px * np.cos(th))

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            if math.hypot(mx - x_bob_px, my - y_bob_px) < 26:
                self.dragging_bob = True
                dx = mx - x_c_px
                dy = self.rail_y - my
                self.mouse_target_th = math.atan2(dx, dy)
            cart_rect = pygame.Rect(x_c_px - self.cart_w_px//2, self.rail_y - self.cart_h_px//2,
                                    self.cart_w_px, self.cart_h_px)
            if cart_rect.collidepoint(mx, my):
                self.dragging_cart = True
                self.mouse_target_x = (mx - self.origin_x) / self.ppm

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging_bob = False
            self.dragging_cart = False

        elif event.type == pygame.MOUSEMOTION:
            mx, my = event.pos
            if self.dragging_bob:
                dx = mx - x_c_px
                dy = self.rail_y - my
                self.mouse_target_th = math.atan2(dx, dy)
            elif self.dragging_cart:
                self.mouse_target_x = np.clip((mx - self.origin_x) / self.ppm, -self.cfg.x_rail, self.cfg.x_rail)

    # -------------------------------------------------------------------------
    # Візуалізація
    # -------------------------------------------------------------------------
    def _render(self):
        self.screen.fill((18, 22, 30))

        # 1. Рейка
        x_rail = self.cfg.x_rail
        rail_start_x = int(self.origin_x - (x_rail + 0.2) * self.ppm)
        rail_end_x   = int(self.origin_x + (x_rail + 0.2) * self.ppm)

        pygame.draw.line(self.screen, (45, 52, 70), (rail_start_x, self.rail_y), (rail_end_x, self.rail_y), 8)
        pygame.draw.line(self.screen, (100, 115, 145), (rail_start_x, self.rail_y - 2), (rail_end_x, self.rail_y - 2), 2)

        # Відбійники
        bump_l_x = int(self.origin_x - x_rail * self.ppm)
        bump_r_x = int(self.origin_x + x_rail * self.ppm)
        for bx in [bump_l_x, bump_r_x]:
            pygame.draw.line(self.screen, (220, 50, 60), (bx, self.rail_y - 25), (bx, self.rail_y + 25), 4)
            sign = 1 if bx == bump_l_x else -1
            pygame.draw.rect(self.screen, (180, 40, 50), (bx - (14 if sign==1 else 0), self.rail_y - 10, 14, 20), border_radius=2)

        # Мітки дистанції
        step_m = 0.25 if x_rail <= 0.6 else 0.5
        for mark_m in np.arange(-x_rail, x_rail + 0.01, step_m):
            mx_px = int(self.origin_x + mark_m * self.ppm)
            pygame.draw.line(self.screen, (65, 75, 95), (mx_px, self.rail_y - 5), (mx_px, self.rail_y + 5), 2)
            lbl = self.font_small.render(f"{mark_m:+.2f}м" if abs(mark_m) > 0.01 else " 0.0м", True, (120, 135, 160))
            self.screen.blit(lbl, (mx_px - 16, self.rail_y + 10))

        # 2. Вагонетка (Візок)
        x_c_px = int(self.origin_x + self.state[0] * self.ppm)
        cart_rect = pygame.Rect(x_c_px - self.cart_w_px//2, self.rail_y - self.cart_h_px//2,
                                self.cart_w_px, self.cart_h_px)
        cart_col = (45, 95, 155) if not self.dragging_cart else (70, 130, 205)
        pygame.draw.rect(self.screen, cart_col, cart_rect, border_radius=6)
        pygame.draw.rect(self.screen, (130, 175, 230), cart_rect, width=2, border_radius=6)

        # Колеса
        wheel_r = 9
        w_offset = self.cart_w_px // 3
        for w_pos in [(x_c_px - w_offset, self.rail_y + self.cart_h_px//2 - 2),
                      (x_c_px + w_offset, self.rail_y + self.cart_h_px//2 - 2)]:
            pygame.draw.circle(self.screen, (25, 28, 36), w_pos, wheel_r)
            pygame.draw.circle(self.screen, (160, 170, 185), w_pos, wheel_r - 3)
            pygame.draw.circle(self.screen, (40, 45, 55), w_pos, 2)

        # 3. Стрілка сили мотора (u)
        cur_umax = (self.cfg.M + self.cfg.m) * self.slider_amax.val
        if abs(self.last_u) > 0.4:
            force_len = int((self.last_u / max(1.0, cur_umax)) * 75)
            arrow_col = (50, 205, 100) if self.last_u > 0 else (240, 140, 40)
            y_arr = self.rail_y - 25
            pygame.draw.line(self.screen, arrow_col, (x_c_px, y_arr), (x_c_px + force_len, y_arr), 4)
            tip_x = x_c_px + force_len
            sign = 1 if force_len > 0 else -1
            pygame.draw.polygon(self.screen, arrow_col, [
                (tip_x, y_arr),
                (tip_x - sign * 8, y_arr - 5),
                (tip_x - sign * 8, y_arr + 5)
            ])

        # 4. Маятник (Стержень + Вантаж)
        th = self.state[1]
        x_bob_px = int(x_c_px + self.rod_len_px * np.sin(th))
        y_bob_px = int(self.rail_y - self.rod_len_px * np.cos(th))

        pygame.draw.line(self.screen, (225, 60, 45), (x_c_px, self.rail_y), (x_bob_px, y_bob_px), 5)
        bob_radius = 15
        bob_col = (210, 40, 30) if not self.dragging_bob else (255, 100, 80)
        pygame.draw.circle(self.screen, bob_col, (x_bob_px, y_bob_px), bob_radius)
        pygame.draw.circle(self.screen, (255, 255, 255), (x_bob_px, y_bob_px), bob_radius, width=2)
        pygame.draw.circle(self.screen, (255, 150, 140), (x_bob_px - 3, y_bob_px - 3), 4)

        # Шарнір
        pygame.draw.circle(self.screen, (255, 255, 255), (x_c_px, self.rail_y), 5)
        pygame.draw.circle(self.screen, (20, 25, 35), (x_c_px, self.rail_y), 2)

        # 5. UI Панелі
        self._render_header()
        self._render_oscilloscope()
        self._render_controls()

    def _render_header(self):
        title_surf = self.font_title.render("SINGLE INVERTED PENDULUM (MPC SWING-UP + LQR)", True, (240, 245, 255))
        self.screen.blit(title_surf, (30, 16))

        hint_surf = self.font_small.render("Гарячі клавіші: M: MPC Swing-Up | E: Енерго Swing-Up | Space: Мотор | D: Вниз | R: Скинути | F11: Екран", True, (135, 150, 175))
        self.screen.blit(hint_surf, (30, 42))

        th_deg = np.degrees((self.state[1] + np.pi) % (2.0 * np.pi) - np.pi)
        x_m = self.state[0]

        if not self.motor_enabled:
            status_text = "МОТОР ВИМКНЕНО (Вільне хитання)"
            status_col = (140, 145, 160)
        elif self.current_mode == "MPC_SWINGUP":
            prog = self.mpc.get_progress() * 100.0
            status_text = f"▶ РОЗГІН MPC (Swing-Up: {prog:.0f}%, t={self.mpc.playback_time:.2f}c / {self.mpc.duration:.1f}c)"
            status_col = (55, 180, 245)
        elif self.current_mode == "LQR":
            if abs(th_deg) < 2.5 and abs(x_m) < 0.08:
                status_text = "● СТАБІЛІЗОВАНО (LQR Балансування)"
                status_col = (50, 205, 100)
            else:
                status_text = "⚡ СТАБІЛІЗАЦІЯ (LQR перехопив керування)"
                status_col = (245, 155, 45)
        elif self.current_mode == "ENERGY_SWINGUP":
            status_text = "🔄 ЕНЕРГЕТИЧНИЙ ПІДЙОМ (Åström / Spong)"
            status_col = (235, 130, 45)
        else:
            status_text = "ОЧІКУВАННЯ / НИЖНЄ ПОЛОЖЕННЯ"
            status_col = (175, 180, 195)

        badge_surf = self.font_title.render(status_text, True, status_col)
        self.screen.blit(badge_surf, (30, 68))

        # Телеметрія
        card_w = 320
        card_h = 100
        card_x = min(self.width - 440, max(490, self.origin_x + 30))
        card_rect = pygame.Rect(card_x, 14, card_w, card_h)
        pygame.draw.rect(self.screen, (25, 30, 42), card_rect, border_radius=6)
        pygame.draw.rect(self.screen, (45, 54, 72), card_rect, width=1, border_radius=6)

        w_deg = np.degrees(self.state[3])
        v_m = self.state[2]
        fps_val = int(self.clock.get_fps())
        cur_amax = self.slider_amax.val
        cur_vmax = self.slider_vmax.val

        info_lines = [
            f"Кут θ: {th_deg:+5.1f}°  (ω: {w_deg:+5.1f}°/с)",
            f"Позиція x: {x_m:+5.2f} м  (v: {v_m:+5.2f} м/с, ліміт: {cur_vmax:.2f})",
            f"Прискорення a: {self.last_a:+5.1f} м/с² (ліміт: {cur_amax:.1f})",
            f"Сила мотора u: {self.last_u:+5.1f} Н | FPS: {fps_val}"
        ]
        for i, line in enumerate(info_lines):
            t_surf = self.font_main.render(line, True, (210, 225, 245))
            self.screen.blit(t_surf, (card_x + 14, 18 + i * 20))

    def _render_oscilloscope(self):
        ow = 240
        oh = 95
        ox = self.width - ow - 25
        oy = 60

        pygame.draw.rect(self.screen, (25, 30, 42), (ox, oy, ow, oh), border_radius=6)
        pygame.draw.rect(self.screen, (50, 60, 80), (ox, oy, ow, oh), width=1, border_radius=6)

        mid_y = oy + oh // 2
        band_h = int((3.0 / 40.0) * (oh // 2))
        pygame.draw.rect(self.screen, (30, 70, 45), (ox + 1, mid_y - band_h, ow - 2, band_h * 2))
        pygame.draw.line(self.screen, (60, 70, 90), (ox, mid_y), (ox + ow, mid_y), 1)

        title = self.font_small.render("Осцилограф кута θ(t)", True, (150, 165, 190))
        self.screen.blit(title, (ox + 8, oy + 5))

        if len(self.theta_history) > 1:
            points = []
            dx = ow / float(self.max_hist_len)
            for i, val in enumerate(self.theta_history):
                px = ox + int(i * dx)
                py = mid_y - int((val / 40.0) * (oh // 2 - 8))
                py = max(oy + 2, min(oy + oh - 2, py))
                points.append((px, py))
            if len(points) >= 2:
                pygame.draw.lines(self.screen, (235, 75, 55), False, points, 2)

    def _render_controls(self):
        panel_y = max(490, self.height - 200)
        panel_w = self.width - 60
        panel_h = 185

        pygame.draw.rect(self.screen, (24, 28, 38), (30, panel_y, panel_w, panel_h), border_radius=8)
        pygame.draw.rect(self.screen, (45, 52, 70), (30, panel_y, panel_w, panel_h), width=1, border_radius=8)

        panel_title = self.font_title.render("ПАРАМЕТРИ LQR ТА ОБМЕЖЕННЯ ПРИВОДУ (L = 0.35 м, A_MAX = 15.0 м/с², V_MAX = 0.66 м/с)", True, (210, 225, 245))
        self.screen.blit(panel_title, (45, panel_y + 14))

        for slider in self.sliders:
            slider.draw(self.screen, self.font_main)
        for button in self.buttons:
            button.draw(self.screen, self.font_main)


if __name__ == '__main__':
    sim = RealtimeCartPoleSim()
    sim.run()
