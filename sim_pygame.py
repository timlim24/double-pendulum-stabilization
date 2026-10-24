"""
=============================================================================
РЕАЛЬНОЧАСОВИЙ (REALTIME 60 FPS) СИМУЛЯТОР СТАБІЛІЗАЦІЇ ТА ПІДЙОМУ (SWING-UP)
=============================================================================
Бібліотека: Pygame
Оновлені можливості:
1. ПОВНИЙ ЕКРАН / МАСШТАБУВАННЯ (F11 або кнопка '⛶ Екран', підтримує будь-яку роздільну здатність).
2. АВТОМАТИЧНИЙ ПІДЙОМ (SWING-UP): якщо опустити маятник донизу, візок розгойдує
   його за допомогою енергетичного керування (Åström / Spong), піднімає та ловить у вертикаль!
3. ПРАВИЛЬНА ІНЕРЦІЙНА ФІЗИКА МИШКИ: при русі візка маятник відхиляється від інерції,
   як у реальному світі (пружинно-демпферне захоплення).
4. Інтерактивні слайдери для LQR-констант прямо на екрані.
5. Осцилограф кута в реальному часі.
=============================================================================
"""

import sys
import math
import numpy as np
import scipy.linalg
import pygame

# =============================================================================
# 1. ФІЗИЧНІ ПАРАМЕТРИ РЕАЛЬНОЇ УСТАНОВКИ
# =============================================================================
M = 1.0          # Маса вагонетки/візка (кг)
m = 0.1          # Маса маятника (стержня) (кг)
L = 0.35         # Загальна довжина стержня маятника (метри)
l = L / 2.0      # Відстань до центру мас (м) [для однорідного = L/2]
g = 9.81         # Гравітація (м/с^2)
I = (4.0 / 3.0) * m * (l ** 2) # Момент інерції відносно шарніра

X_RAIL = .5     # Межа рейки від центру в один бік (повна довжина рейки = 4.4 м)

# Реалістичні обмеження приводу (як у реальних серво/крокових установках)
A_MAX_DEFAULT = 15.0   # Максимальне прискорення візка (м/с^2) [~1.83g]
V_MAX_DEFAULT = 0.66   # Максимальна лінійна швидкість візка (м/с)
U_MAX_DEFAULT = (M + m) * A_MAX_DEFAULT # Максимальне номінальне зусилля мотора (Ньютони, ~19.8 Н)
U_MAX = U_MAX_DEFAULT

B_CART = 0.15    # Тертя візка об рейку (Н*с/м)
B_PEND = 0.005   # Тертя в підшипнику маятника (Н*м*с/рад)

K_BUMPER = 5000.0  # Жорсткість відбійників (Н/м)
D_BUMPER = 120.0   # Демпфування відбійників (Н*с/м)

# Початковий кут нахилу
INIT_THETA_DEG = 15.0


# =============================================================================
# 2. АВТОМАТИЧНИЙ РОЗРАХУНОК ОПТИМАЛЬНИХ КОНСТАНТ (LQR)
# =============================================================================
def calculate_optimal_lqr(M_val, m_val, l_val, g_val,
                          q_x=15.0, q_theta=160.0, q_v=6.0, q_w=30.0, r_effort=0.08):
    denom = l_val * (4.0 * M_val + m_val)
    a32 = -3.0 * m_val * g_val / (4.0 * M_val + m_val)
    a42 = 3.0 * (M_val + m_val) * g_val / denom
    b3  = 4.0 / (4.0 * M_val + m_val)
    b4  = -3.0 / denom

    A = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, a32, 0.0, 0.0],
        [0.0, a42, 0.0, 0.0]
    ])
    B = np.array([[0.0], [0.0], [b3], [b4]])
    Q = np.diag([q_x, q_theta, q_v, q_w])
    R = np.array([[r_effort]])

    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K_raw = (np.linalg.inv(R) @ B.T @ P).flatten()

    k_x_opt     = -K_raw[0]
    k_theta_opt = -K_raw[1]
    k_v_opt     = -K_raw[2]
    k_omega_opt = -K_raw[3]

    return k_theta_opt, k_omega_opt, k_x_opt, k_v_opt


k_th_lqr, k_om_lqr, k_x_lqr, k_v_lqr = calculate_optimal_lqr(M, m, l, g)


# =============================================================================
# 3. ЕЛЕМЕНТИ ІНТЕРФЕЙСУ (СЛАЙДЕРИ ТА КНОПКИ)
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
        
        rel = (self.val - self.min_val) / (self.max_val - self.min_val)
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
# 4. ГОЛОВНИЙ КЛАС РЕАЛЬНОЧАСОВОГО СИМУЛЯТОРА
# =============================================================================
class RealtimeCartPoleSim:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Стабілізація та Swing-Up маятника — Pygame 60 FPS")
        
        self.width = 1200
        self.height = 750
        self.is_fullscreen = False
        
        # Режим вікна з підтримкою зміни розміру
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()
        
        self.font_title = pygame.font.SysFont("dejavusans", 16, bold=True)
        self.font_main  = pygame.font.SysFont("dejavusans", 13)
        self.font_small = pygame.font.SysFont("dejavusans", 11)
        
        # Стан системи: [x, theta, v, omega]
        self.state = np.array([0.0, np.radians(INIT_THETA_DEG), 0.0, 0.0], dtype=float)
        self.motor_enabled = True
        self.last_u = 0.0
        self.last_a = 0.0
        self.current_mode = "LQR"
        self.swing_kick_dir = 1.0
        
        # Миша: взаємодія та пружинне захоплення
        self.dragging_bob = False
        self.dragging_cart = False
        self.mouse_target_x = 0.0
        self.mouse_target_th = 0.0
        
        # Історія для осцилографа
        self.theta_history = []
        self.max_hist_len = 220
        
        # Оновлення динамічного розташування та створення UI
        self._update_layout()

    def _update_layout(self):
        # Масштаб сцени під поточний розмір вікна
        self.origin_x = self.width // 2
        self.rail_y = int(self.height * 0.46)
        
        # Довжина рейки займає 72% ширини екрана
        available_w = max(600, self.width - 240)
        self.ppm = available_w / (2.0 * (X_RAIL + 0.35))
        
        self.cart_w_px = max(60, int(0.36 * self.ppm))
        self.cart_h_px = max(24, int(0.16 * self.ppm))
        self.rod_len_px = int(L * self.ppm)
        
        # Нижня панель UI
        panel_y = max(480, self.height - 190)
        slider_w = min(210, max(140, int(self.width * 0.17)))
        slider_h = 10
        gap = min(35, max(15, int(self.width * 0.025)))
        
        col1_x = 40
        col2_x = col1_x + slider_w + gap
        col3_x = col2_x + slider_w + gap
        
        # Зберігаємо попередні значення якщо слайдери вже були створені
        v_kth  = getattr(self, 'slider_kth', None).val if hasattr(self, 'slider_kth') else k_th_lqr
        v_kom  = getattr(self, 'slider_kom', None).val if hasattr(self, 'slider_kom') else k_om_lqr
        v_kx   = getattr(self, 'slider_kx', None).val if hasattr(self, 'slider_kx') else k_x_lqr
        v_kv   = getattr(self, 'slider_kv', None).val if hasattr(self, 'slider_kv') else k_v_lqr
        v_amax = getattr(self, 'slider_amax', None).val if hasattr(self, 'slider_amax') else A_MAX_DEFAULT
        v_vmax = getattr(self, 'slider_vmax', None).val if hasattr(self, 'slider_vmax') else V_MAX_DEFAULT
        
        self.slider_kth  = Slider(col1_x, panel_y + 55,  slider_w, slider_h, 0.0, 350.0, v_kth,  "k_θ (кут)", color=(230, 80, 60))
        self.slider_kom  = Slider(col1_x, panel_y + 115, slider_w, slider_h, 0.0, 100.0, v_kom,  "k_ω (демпфер)", color=(235, 130, 45))
        self.slider_kx   = Slider(col2_x, panel_y + 55,  slider_w, slider_h, 0.0, 60.0,  v_kx,   "k_x (рейка)", color=(50, 150, 235))
        self.slider_kv   = Slider(col2_x, panel_y + 115, slider_w, slider_h, 0.0, 60.0,  v_kv,   "k_v (швидкість)", color=(60, 190, 160))
        self.slider_amax = Slider(col3_x, panel_y + 55,  slider_w, slider_h, 5.0, 45.0,  v_amax, "a_max (макс a)", unit=" м/с²", color=(185, 80, 220))
        self.slider_vmax = Slider(col3_x, panel_y + 115, slider_w, slider_h, 1.0, 6.0,   v_vmax, "v_max (макс v)", unit=" м/с", color=(215, 165, 45))
        
        self.sliders = [self.slider_kth, self.slider_kom, self.slider_kx, self.slider_kv, self.slider_amax, self.slider_vmax]
        
        # Кнопки
        btn_w = min(140, max(110, int(self.width * 0.11)))
        btn_h = 40
        btn_col1 = col3_x + slider_w + gap
        btn_col2 = btn_col1 + btn_w + 12
        
        self.btn_lqr   = Button(btn_col1, panel_y + 40, btn_w, btn_h, "★ Авто LQR", (35, 95, 165), (50, 125, 210), self.set_lqr)
        self.btn_motor = Button(btn_col1, panel_y + 95, btn_w, btn_h, "Мотор: УВІМК" if self.motor_enabled else "Мотор: ВИМК",
                                (40, 130, 85) if self.motor_enabled else (150, 45, 45),
                                (55, 165, 110) if self.motor_enabled else (190, 60, 60), self.toggle_motor)
        self.btn_reset = Button(btn_col2, panel_y + 40, btn_w, btn_h, "↻ Скинути", (105, 110, 125), (130, 135, 150), self.reset_state)
        self.btn_drop  = Button(btn_col2, panel_y + 95, btn_w, btn_h, "⬇ Опустити вниз", (160, 80, 40), (200, 105, 55), self.drop_down)
        
        # Кнопка перемикання повного екрана (у верхньому правому кутку)
        self.btn_fullscreen = Button(self.width - 130, 16, 100, 32, "⛶ Екран", (45, 55, 75), (65, 80, 105), self.toggle_fullscreen)
        
        self.buttons = [self.btn_lqr, self.btn_motor, self.btn_reset, self.btn_drop, self.btn_fullscreen]

    def toggle_fullscreen(self):
        self.is_fullscreen = not self.is_fullscreen
        if self.is_fullscreen:
            info = pygame.display.Info()
            self.width, self.height = info.current_w, info.current_h
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
        else:
            self.width, self.height = 1200, 750
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self._update_layout()

    def set_lqr(self):
        self.slider_kth.val = k_th_lqr
        self.slider_kom.val = k_om_lqr
        self.slider_kx.val  = k_x_lqr
        self.slider_kv.val  = k_v_lqr
        if hasattr(self, 'slider_amax'):
            self.slider_amax.val = A_MAX_DEFAULT
        if hasattr(self, 'slider_vmax'):
            self.slider_vmax.val = V_MAX_DEFAULT

    def toggle_motor(self):
        self.motor_enabled = not self.motor_enabled
        if self.motor_enabled:
            self.btn_motor.text = "Мотор: УВІМК"
            self.btn_motor.color = (40, 130, 85)
            self.btn_motor.hover_color = (55, 165, 110)
        else:
            self.btn_motor.text = "Мотор: ВИМК"
            self.btn_motor.color = (150, 45, 45)
            self.btn_motor.hover_color = (190, 60, 60)

    def reset_state(self):
        self.state = np.array([0.0, np.radians(INIT_THETA_DEG), 0.0, 0.0], dtype=float)

    def drop_down(self):
        """Опускає маятник у нижнє положення, щоб протестувати автоматичний підйом (Swing-Up)"""
        self.state = np.array([0.0, np.pi, 0.0, 0.0], dtype=float)
        self.swing_kick_dir = 1.0

    # -------------------------------------------------------------------------
    # Розрахунок комбінованого керування (LQR + Енергетичний Swing-Up)
    # -------------------------------------------------------------------------
    def compute_control_force(self, s):
        if not self.motor_enabled:
            return 0.0, "OFF"
            
        x_val, th_val, v_val, w_val = s
        th_err = (th_val + np.pi) % (2.0 * np.pi) - np.pi
        
        # Динамічні межі приводу від налаштувань користувача
        a_max = self.slider_amax.val if hasattr(self, 'slider_amax') else A_MAX_DEFAULT
        v_max = self.slider_vmax.val if hasattr(self, 'slider_vmax') else V_MAX_DEFAULT
        u_max = (M + m) * a_max
        
        # 1. ЗОНА ЗАХОПЛЕННЯ LQR: відхилення кута < 28° та помірна кутова швидкість
        catch_angle_rad = np.radians(28.0)
        if abs(th_err) < catch_angle_rad and abs(w_val) < 4.2:
            k_th = self.slider_kth.val
            k_om = self.slider_kom.val
            k_pos = self.slider_kx.val
            k_vel = self.slider_kv.val
            
            u = k_th * th_err + k_om * w_val + k_pos * x_val + k_vel * v_val
            mode = "LQR"
        else:
            # 2. РЕЖИМ ПІДЙОМУ (SWING-UP ЗА МЕХАНІЧНОЮ ЕНЕРГІЄЮ)
            # Повна енергія відносно верхньої точки (у верхній точці E = 0, внизу E < 0)
            E = 0.5 * I * (w_val ** 2) + m * g * l * (np.cos(th_err) - 1.0)
            
            # Якщо маятник завис нерухомо біля нижньої мертвої точки, даємо плавний стартовий імпульс
            # Фіксуємо напрямок імпульсу (latch), доки маятник не набере достатню швидкість (|w| >= 0.4 рад/с)
            if abs(th_err) > 2.2 and abs(w_val) < 0.4:
                kick_force = min(u_max * 0.9, 24.0)
                u_pump = kick_force * self.swing_kick_dir
            else:
                # Оновлюємо напрямок наступного старту тільки якщо візок помітно змістився від центру
                if abs(x_val) > 0.4:
                    self.swing_kick_dir = -1.0 if x_val > 0 else 1.0
                # Фізично строгий неперервний закон накачування енергії (Åström / Spong):
                # dE/dt = -m * l * w * a_cart * cos(theta).
                # Оскільки E < 0 нижче вершини, а в нижній півплощині cos(theta) < 0:
                # при w > 0 добуток E * w * cos > 0, отже візок прискорюється в напрямку w,
                # резонансно підсилюючи розгойдування кожного півперіоду без ривків і брязкоту!
                k_e = 75.0
                u_pump = k_e * E * w_val * np.cos(th_err)
            
            # Центрування візка до середини рейки (x = 0)
            u_center = -14.0 * x_val - 8.0 * v_val
            
            # М'який захисний бар'єр біля країв рейки (починає діяти з 75% довжини рейки)
            edge_zone = X_RAIL * 0.75
            if abs(x_val) > edge_zone:
                excess = abs(x_val) - edge_zone
                u_edge = -40.0 * np.sign(x_val) * excess
            else:
                u_edge = 0.0
                
            u = u_pump + u_center + u_edge
            mode = "SWING-UP"
            
        # Обмеження сили мотора за максимальним прискоренням a_max
        u = float(np.clip(u, -u_max, u_max))
        
        # Обмеження за максимальною лінійною швидкістю v_max (проти-ЕРС реального електродвигуна)
        if v_val >= v_max and u > 0:
            u = 0.0
        elif v_val <= -v_max and u < 0:
            u = 0.0
            
        return u, mode

    # -------------------------------------------------------------------------
    # Фізичний рушій (Нелінійна динаміка системи візок + маятник)
    # -------------------------------------------------------------------------
    def dynamics(self, s, external_cart_force=0.0):
        x_val, th_val, v_val, w_val = s
        
        # Керуюча сила мотора
        u, mode = self.compute_control_force(s)
        self.last_u = u
        self.current_mode = mode
        
        # Додаємо зовнішню силу (наприклад, пружина від мишки при перетягуванні)
        total_force = u + external_cart_force
        
        sin_t = np.sin(th_val)
        cos_t = np.cos(th_val)
        
        denom = l * (4.0 / 3.0 - m * (cos_t ** 2) / (M + m))
        temp = (total_force - B_CART * v_val + m * l * (w_val ** 2) * sin_t) / (M + m)
        
        theta_acc = (g * sin_t - cos_t * temp - (B_PEND / (m * l)) * w_val) / denom
        x_acc = temp - (m * l * theta_acc * cos_t) / (M + m)
        
        # Фізичне обмеження прискорення візка від приводу (a <= a_max)
        a_max = self.slider_amax.val if hasattr(self, 'slider_amax') else A_MAX_DEFAULT
        if abs(x_acc) > a_max:
            x_acc = float(np.clip(x_acc, -a_max, a_max))
            # Маятник відчуває реальне прискорення осі (принцип Д'Аламбера)
            theta_acc = (g * sin_t - x_acc * cos_t - (B_PEND / (m * l)) * w_val) / (l * (4.0 / 3.0))
            
        self.last_a = x_acc
        
        # Пружні гумові відбійники рейки (зовнішня реакція стінки при виході за межі)
        if x_val >= X_RAIL:
            f_bump = -K_BUMPER * (x_val - X_RAIL) - D_BUMPER * max(0.0, v_val)
            x_acc += f_bump / M
        elif x_val <= -X_RAIL:
            f_bump = -K_BUMPER * (x_val + X_RAIL) - D_BUMPER * min(0.0, v_val)
            x_acc += f_bump / M
            
        return np.array([v_val, w_val, x_acc, theta_acc])

    def rk4_step(self, dt, ext_force=0.0):
        k1 = self.dynamics(self.state, ext_force)
        k2 = self.dynamics(self.state + 0.5 * dt * k1, ext_force)
        k3 = self.dynamics(self.state + 0.5 * dt * k2, ext_force)
        k4 = self.dynamics(self.state + dt * k3, ext_force)
        self.state += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    # -------------------------------------------------------------------------
    # Головний цикл симуляції
    # -------------------------------------------------------------------------
    def run(self):
        running = True
        while running:
            # Обчислення дельти часу (60 FPS)
            raw_dt = self.clock.tick(60) / 1000.0
            dt = min(0.04, max(0.001, raw_dt))
            
            # Обробка подій
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
                    elif event.key == pygame.K_r:
                        self.reset_state()
                    elif event.key == pygame.K_a:
                        self.set_lqr()
                    elif event.key == pygame.K_d:
                        self.drop_down()
                    elif event.key == pygame.K_LEFT:
                        self.state[2] -= 1.5
                    elif event.key == pygame.K_RIGHT:
                        self.state[2] += 1.5
                        
                # Події UI
                ui_captured = False
                for slider in self.sliders:
                    if slider.handle_event(event):
                        ui_captured = True
                for button in self.buttons:
                    if button.handle_event(event):
                        ui_captured = True
                        
                # Миша: взаємодія зі сценою
                if not ui_captured:
                    self._handle_mouse_physics(event)
                    
            # Розрахунок фізичного впливу від перетягування мишкою
            ext_cart_force = 0.0
            if self.dragging_cart:
                # Пружинно-демпферне захоплення візка мишкою
                # Візок відчуває реальну силу тяги, а маятник — реальну інерцію!
                k_drag = 700.0
                d_drag = 65.0
                cart_x = self.state[0]
                cart_v = self.state[2]
                ext_cart_force = k_drag * (self.mouse_target_x - cart_x) - d_drag * cart_v
                
            # Фізична інтеграція (8 мікрокроків на кадр)
            if not self.dragging_bob:
                sub_steps = 8
                dt_sub = dt / sub_steps
                for _ in range(sub_steps):
                    self.rk4_step(dt_sub, ext_cart_force)
            else:
                # Якщо тримаємо маятник руками за вантаж:
                # плавно підтягуємо кут до положення курсора
                th_diff = (self.mouse_target_th - self.state[1] + np.pi) % (2.0 * np.pi) - np.pi
                self.state[1] += th_diff * 0.35
                self.state[3] = th_diff / dt # Реалістична швидкість при відпусканні
                
            # Запис в осцилограф
            th_deg = np.degrees((self.state[1] + np.pi) % (2.0 * np.pi) - np.pi)
            self.theta_history.append(th_deg)
            if len(self.theta_history) > self.max_hist_len:
                self.theta_history.pop(0)
                
            # Рендеринг кадру
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
            # Клік по вантажу маятника
            if math.hypot(mx - x_bob_px, my - y_bob_px) < 26:
                self.dragging_bob = True
                dx = mx - x_c_px
                dy = self.rail_y - my
                self.mouse_target_th = math.atan2(dx, dy)
            # Клік по візку
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
                self.mouse_target_x = np.clip((mx - self.origin_x) / self.ppm, -X_RAIL, X_RAIL)

    # -------------------------------------------------------------------------
    # Візуалізація сцени
    # -------------------------------------------------------------------------
    def _render(self):
        self.screen.fill((18, 22, 30))
        
        # 1. Рейка
        rail_start_x = int(self.origin_x - (X_RAIL + 0.3) * self.ppm)
        rail_end_x   = int(self.origin_x + (X_RAIL + 0.3) * self.ppm)
        
        pygame.draw.line(self.screen, (45, 52, 70), (rail_start_x, self.rail_y), (rail_end_x, self.rail_y), 8)
        pygame.draw.line(self.screen, (100, 115, 145), (rail_start_x, self.rail_y - 2), (rail_end_x, self.rail_y - 2), 2)
        
        # Відбійники
        bump_l_x = int(self.origin_x - X_RAIL * self.ppm)
        bump_r_x = int(self.origin_x + X_RAIL * self.ppm)
        
        for bx in [bump_l_x, bump_r_x]:
            pygame.draw.line(self.screen, (220, 50, 60), (bx, self.rail_y - 30), (bx, self.rail_y + 30), 4)
            sign = 1 if bx == bump_l_x else -1
            pygame.draw.rect(self.screen, (180, 40, 50), (bx - (15 if sign==1 else 0), self.rail_y - 12, 15, 24), border_radius=2)

        # Мітки дистанції вздовж рейки
        step_m = 0.5
        max_mark = int(X_RAIL / step_m) * step_m
        for mark_m in np.arange(-max_mark, max_mark + 0.1, step_m):
            mx_px = int(self.origin_x + mark_m * self.ppm)
            pygame.draw.line(self.screen, (65, 75, 95), (mx_px, self.rail_y - 6), (mx_px, self.rail_y + 6), 2)
            lbl = self.font_small.render(f"{mark_m:+.1f}м" if abs(mark_m) > 0.01 else " 0.0м", True, (120, 135, 160))
            self.screen.blit(lbl, (mx_px - 14, self.rail_y + 12))

        # 2. Вагонетка (Візок)
        x_c_px = int(self.origin_x + self.state[0] * self.ppm)
        cart_rect = pygame.Rect(x_c_px - self.cart_w_px//2, self.rail_y - self.cart_h_px//2,
                                self.cart_w_px, self.cart_h_px)
        
        cart_col = (45, 95, 155) if not self.dragging_cart else (70, 130, 205)
        pygame.draw.rect(self.screen, cart_col, cart_rect, border_radius=6)
        pygame.draw.rect(self.screen, (130, 175, 230), cart_rect, width=2, border_radius=6)
        
        # Колеса
        wheel_r = 11
        w_offset = self.cart_w_px // 3
        wheel_l_pos = (x_c_px - w_offset, self.rail_y + self.cart_h_px//2 - 2)
        wheel_r_pos = (x_c_px + w_offset, self.rail_y + self.cart_h_px//2 - 2)
        
        for w_pos in [wheel_l_pos, wheel_r_pos]:
            pygame.draw.circle(self.screen, (25, 28, 36), w_pos, wheel_r)
            pygame.draw.circle(self.screen, (160, 170, 185), w_pos, wheel_r - 4)
            pygame.draw.circle(self.screen, (40, 45, 55), w_pos, 3)

        # 3. Стрілка сили мотора (u)
        cur_amax = self.slider_amax.val if hasattr(self, 'slider_amax') else A_MAX_DEFAULT
        cur_umax = (M + m) * cur_amax
        if abs(self.last_u) > 0.5:
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
        
        bob_radius = 16
        bob_col = (210, 40, 30) if not self.dragging_bob else (255, 100, 80)
        pygame.draw.circle(self.screen, bob_col, (x_bob_px, y_bob_px), bob_radius)
        pygame.draw.circle(self.screen, (255, 255, 255), (x_bob_px, y_bob_px), bob_radius, width=2)
        pygame.draw.circle(self.screen, (255, 150, 140), (x_bob_px - 3, y_bob_px - 3), 4)
        
        # Шарнір
        pygame.draw.circle(self.screen, (255, 255, 255), (x_c_px, self.rail_y), 5)
        pygame.draw.circle(self.screen, (20, 25, 35), (x_c_px, self.rail_y), 2)

        # 5. Інтерфейсні панелі
        self._render_header()
        self._render_oscilloscope()
        self._render_controls()

    def _render_header(self):
        # Ліва сторона: Заголовок та підказки
        title_surf = self.font_title.render("CART-POLE REALTIME SIMULATOR (60 FPS)", True, (240, 245, 255))
        self.screen.blit(title_surf, (30, 16))
        
        hint_surf = self.font_small.render("Мишкою: тягати маятник/візок | Слайдери: K та a_max, v_max | Пробіл: мотор | D: опустити | F11: екран", True, (135, 150, 175))
        self.screen.blit(hint_surf, (30, 42))
        
        # Статус системи та режим роботи
        th_deg = np.degrees((self.state[1] + np.pi) % (2.0 * np.pi) - np.pi)
        x_m = self.state[0]
        
        if not self.motor_enabled:
            status_text = "МОТОР ВИМКНЕНО (Вільне хитання)"
            status_col = (140, 145, 160)
        elif self.current_mode == "LQR":
            if abs(th_deg) < 2.5 and abs(x_m) < 0.1:
                status_text = "● СТАБІЛІЗОВАНО (LQR Балансування)"
                status_col = (50, 205, 100)
            else:
                status_text = "⚡ СТАБІЛІЗАЦІЯ (LQR)"
                status_col = (245, 155, 45)
        elif self.current_mode == "SWING-UP":
            status_text = "🔄 ПІДЙОМ МАЯТНИКА (SWING-UP В ДІЇ)"
            status_col = (55, 180, 240)
        else:
            status_text = "✕ НЕСТІЙКИЙ СТАН"
            status_col = (235, 60, 60)
            
        badge_surf = self.font_title.render(status_text, True, status_col)
        self.screen.blit(badge_surf, (30, 68))
        
        # Телеметрія: картка праворуч від центра маятника
        card_w = 300
        card_h = 100
        card_x = min(self.width - 430, max(490, self.origin_x + 40))
        card_rect = pygame.Rect(card_x, 14, card_w, card_h)
        pygame.draw.rect(self.screen, (25, 30, 42), card_rect, border_radius=6)
        pygame.draw.rect(self.screen, (45, 54, 72), card_rect, width=1, border_radius=6)
        
        w_deg = np.degrees(self.state[3])
        v_m = self.state[2]
        fps_val = int(self.clock.get_fps())
        cur_amax = self.slider_amax.val if hasattr(self, 'slider_amax') else A_MAX_DEFAULT
        
        info_lines = [
            f"Кут θ: {th_deg:+5.1f}°  (ω: {w_deg:+5.1f}°/с)",
            f"Позиція x: {x_m:+5.2f} м  (v: {v_m:+5.2f} м/с)",
            f"Прискорення a: {self.last_a:+5.1f} м/с² (макс: {cur_amax:.1f})",
            f"Сила мотора u: {self.last_u:+5.1f} Н | FPS: {fps_val}"
        ]
        
        for i, line in enumerate(info_lines):
            t_surf = self.font_main.render(line, True, (210, 225, 245))
            self.screen.blit(t_surf, (card_x + 14, 18 + i * 20))

    def _render_oscilloscope(self):
        # Віконце осцилографа в правому верхньому кутку
        ow = 240
        oh = 95
        ox = self.width - ow - 25
        oy = 60
        
        pygame.draw.rect(self.screen, (25, 30, 42), (ox, oy, ow, oh), border_radius=6)
        pygame.draw.rect(self.screen, (50, 60, 80), (ox, oy, ow, oh), width=1, border_radius=6)
        
        # Цільова смуга стійкості (±3°)
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
        panel_y = max(480, self.height - 190)
        panel_w = self.width - 60
        panel_h = 175
        
        pygame.draw.rect(self.screen, (24, 28, 38), (30, panel_y, panel_w, panel_h), border_radius=8)
        pygame.draw.rect(self.screen, (45, 52, 70), (30, panel_y, panel_w, panel_h), width=1, border_radius=8)
        
        panel_title = self.font_title.render("ПАРАМЕТРИ СТАБІЛІЗАЦІЇ ТА ФІЗИЧНІ ЛІМІТИ ПРИВОДУ (LQR + A_MAX / V_MAX)", True, (210, 225, 245))
        self.screen.blit(panel_title, (50, panel_y + 14))
        
        for slider in self.sliders:
            slider.draw(self.screen, self.font_main)
            
        for button in self.buttons:
            button.draw(self.screen, self.font_main)


if __name__ == '__main__':
    sim = RealtimeCartPoleSim()
    sim.run()
