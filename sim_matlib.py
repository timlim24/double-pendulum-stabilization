"""
=============================================================================
СИМУЛЯТОР СТАБІЛІЗАЦІЇ ПЕРЕВЕРНУТОГО МАЯТНИКА НА ВАГОНЕТЦІ (CART-POLE)
=============================================================================
Даний скрипт моделює реальну фізику візка з маятником та дозволяє:
1. Автоматично розраховувати оптимальні "магічні константи" (LQR регулятор).
2. Задавати та змінювати константи вручну (як у коді, так і інтерактивними повзунками).
3. Спостерігати плавний рух системи в Matplotlib та синхронні графіки параметрів.
4. Отримати готовий C++/Arduino код для прошивки реального мікроконтролера.
=============================================================================
"""

import sys
import numpy as np
import scipy.linalg
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button

# =============================================================================
# 1. ФІЗИЧНІ ПАРАМЕТРИ РЕАЛЬНОЇ УСТАНОВКИ (НАЛАШТУЙТЕ ПІД СВІЙ СТЕНД)
# =============================================================================
M = 1.0          # Маса візка/вагонетки (кг)
m = 0.1          # Маса маятника (стержня) (кг)
L = 1.0          # Загальна довжина стержня маятника (метри)
l = L / 2.0      # Відстань від шарніра до центру мас стержня (м) [для однорідного = L/2]
g = 9.81         # Прискорення вільного падіння (м/с^2)

X_RAIL = 1.5     # Межа рейки від центру в один бік (повна довжина рейки = 2 * X_RAIL = 3.0 м)
U_MAX = 50.0     # Максимальне зусилля мотора вагонетки (Ньютони)

B_CART = 0.15    # Тертя візка об рейку (Н*с/м)
B_PEND = 0.005   # Тертя в осі підшипника маятника (Н*м*с/рад)

# Жорсткість і демпфування кінцевих гумових відбійників рейки
K_BUMPER = 5000.0
D_BUMPER = 120.0

# =============================================================================
# 2. РЕЖИМ РОБОТИ ТА МАГІЧНІ КОНСТАНТИ
# =============================================================================
# True  = автоматично розрахувати оптимальні константи через LQR (рекомендовано)
# False = використовувати ручні константи, вказані нижче
USE_AUTO_LQR = True

# Ручні магічні константи (використовуються, якщо USE_AUTO_LQR = False):
# Керуюче рівняння: u = k_theta * theta + k_omega * omega + k_x * x + k_v * v
K_THETA_MANUAL = 100.0   # Реакція на нахил кута (чим більше, тим різкіше біжить під маятник)
K_OMEGA_MANUAL = 28.0    # Гасіння швидкості падіння (демпфер, усуває розгойдування)
K_X_MANUAL     = 10.0    # Утримання візка біля центру рейки (x = 0)
K_V_MANUAL     = 15.0    # Гасіння лінійної швидкості візка (запобігає розгону візка)

# Початковий стан системи
INIT_THETA_DEG = 15.0    # Початковий кут нахилу маятника (градуси, +15° = нахил праворуч)
INIT_X = 0.0             # Початкове положення візка на рейці (м)

# Параметри симуляції
SIM_DURATION = 8.0       # Тривалість симуляції (секунди)
FPS = 40                 # Кадрів на секунду для анімації


# =============================================================================
# 3. АВТОМАТИЧНИЙ РОЗРАХУНОК ОПТИМАЛЬНИХ КОНСТАНТ (LQR)
# =============================================================================
def calculate_optimal_lqr(M_val, m_val, l_val, g_val,
                          q_x=10.0, q_theta=120.0, q_v=5.0, q_w=25.0, r_effort=0.1):
    """
    Лінеаризує рівняння руху навколо верхньої точки (theta = 0)
    та розв'язує неперервне алгебраїчне рівняння Ріккаті (LQR).
    
    Повертає додатні коефіцієнти: (k_theta, k_omega, k_x, k_v)
    для закону керування:
        u = k_theta * theta + k_omega * omega + k_x * x + k_v * v
    """
    denom = l_val * (4.0 * M_val + m_val)
    a32 = -3.0 * m_val * g_val / (4.0 * M_val + m_val)
    a42 = 3.0 * (M_val + m_val) * g_val / denom
    b3  = 4.0 / (4.0 * M_val + m_val)
    b4  = -3.0 / denom

    # Матриці простору станів (вектор стану: [x, theta, v, omega])
    A = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, a32, 0.0, 0.0],
        [0.0, a42, 0.0, 0.0]
    ])
    B = np.array([[0.0], [0.0], [b3], [b4]])

    # Вагові матриці: Q - штраф за відхилення станів, R - штраф за витрату енергії мотора
    Q = np.diag([q_x, q_theta, q_v, q_w])
    R = np.array([[r_effort]])

    # Розв'язок рівняння Ріккаті
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K_raw = (np.linalg.inv(R) @ B.T @ P).flatten()

    k_x_opt     = -K_raw[0]
    k_theta_opt = -K_raw[1]
    k_v_opt     = -K_raw[2]
    k_omega_opt = -K_raw[3]

    return k_theta_opt, k_omega_opt, k_x_opt, k_v_opt


# Обчислюємо оптимальні константи
k_th_lqr, k_om_lqr, k_x_lqr, k_v_lqr = calculate_optimal_lqr(M, m, l, g)

if USE_AUTO_LQR:
    k_theta_active = k_th_lqr
    k_omega_active = k_om_lqr
    k_x_active     = k_x_lqr
    k_v_active     = k_v_lqr
else:
    k_theta_active = K_THETA_MANUAL
    k_omega_active = K_OMEGA_MANUAL
    k_x_active     = K_X_MANUAL
    k_v_active     = K_V_MANUAL


# =============================================================================
# 4. ДРУК ПІДКАЗОК ТА КОДУ ДЛЯ МІКРОКОНТРОЛЕРА
# =============================================================================
def print_system_info():
    print("=" * 70)
    print(" СИМУЛЯТОР СТАБІЛІЗАЦІЇ МАЯТНИКА НА ВАГОНЕТЦІ (CART-POLE)")
    print("=" * 70)
    print(f"Фізика: Маса візка M = {M} кг, Маса маятника m = {m} кг, Довжина L = {L} м")
    print(f"Математично розраховані ідеальні константи (LQR):")
    print(f"  k_theta (нахил кута)       = {k_th_lqr:8.2f}")
    print(f"  k_omega (кутова швидкість) = {k_om_lqr:8.2f}")
    print(f"  k_x     (позиція візка)    = {k_x_lqr:8.2f}")
    print(f"  k_v     (швидкість візка)  = {k_v_lqr:8.2f}")
    print("-" * 70)
    print("ФОРМУЛА КЕРУВАННЯ ВАГОНЕТКОЮ:")
    print("  u = k_theta * theta + k_omega * omega + k_x * x + k_v * v")
    print("\nЯК ОТРИМАТИ ЗМІННІ З РЕАЛЬНИХ ДАТЧИКІВ:")
    print("  1. theta: безпосередньо з енкодера/потенціометра маятника (у радіанах).")
    print("  2. v:     швидкість вагонетки з енкодера коліс/мотора (м/с).")
    print("  3. omega: похідна кута: omega = (theta - prev_theta) / dt")
    print("  4. x:     інтеграл швидкості: x += v * dt (або підрахунок кроків енкодера)")
    print("=" * 70)
    print("ГОТОВИЙ C++ / ARDUINO КОД ДЛЯ ВАШОГО КОНТРОЛЕРА:")
    print(f"""
// Константи стабілізації:
const float K_THETA = {k_theta_active:.2f}f;
const float K_OMEGA = {k_omega_active:.2f}f;
const float K_X     = {k_x_active:.2f}f;
const float K_V     = {k_v_active:.2f}f;
const float MAX_FORCE = {U_MAX:.1f}f;

float x_pos = 0.0f;
float prev_theta = 0.0f;

void control_loop(float dt) {{
    // 1. Зчитування датчиків:
    float theta = read_pendulum_angle_rad(); // кут у радіанах (0 = строго вгору)
    float v     = read_cart_velocity_mps();  // швидкість візка в м/с
    
    // 2. Обчислення кутової швидкості та позиції:
    float omega = (theta - prev_theta) / dt;
    x_pos += v * dt;
    prev_theta = theta;
    
    // 3. Розрахунок зусилля (магічна формула):
    float u = K_THETA * theta + K_OMEGA * omega + K_X * x_pos + K_V * v;
    
    // 4. Обмеження зусилля мотора:
    if (u > MAX_FORCE)  u = MAX_FORCE;
    if (u < -MAX_FORCE) u = -MAX_FORCE;
    
    // 5. Подача сигналу на мотор (ШІМ / напрямок):
    set_motor_output(u);
}}
""")
    print("=" * 70)


# =============================================================================
# 5. НЕЛІНІЙНА ФІЗИЧНА ДИНАМІКА СИСТЕМИ
# =============================================================================
def simulate_trajectory(k_th, k_om, k_pos, k_vel, init_th_deg, motor_enabled=True):
    """
    Симулює повну нелінійну траєкторію за допомогою чисельного інтегрування RK45.
    """
    total_frames = int(SIM_DURATION * FPS)
    t_eval = np.linspace(0, SIM_DURATION, total_frames)
    
    init_state = [INIT_X, np.radians(init_th_deg), 0.0, 0.0]
    
    def dynamics(t, state):
        x_val, th_val, v_val, w_val = state
        
        # Обчислення керуючого зусилля
        if motor_enabled:
            th_wrapped = (th_val + np.pi) % (2 * np.pi) - np.pi
            if abs(th_wrapped) < np.radians(70.0):
                u = k_th * th_wrapped + k_om * w_val + k_pos * x_val + k_vel * v_val
                u = np.clip(u, -U_MAX, U_MAX)
            else:
                u = 0.0
        else:
            u = 0.0
            
        sin_t = np.sin(th_val)
        cos_t = np.cos(th_val)
        
        # Нелінійні рівняння Лагранжа для системи візок + стержень
        denom = l * (4.0 / 3.0 - m * (cos_t ** 2) / (M + m))
        temp = (u - B_CART * v_val + m * l * (w_val ** 2) * sin_t) / (M + m)
        
        theta_acc = (g * sin_t - cos_t * temp - (B_PEND / (m * l)) * w_val) / denom
        x_acc = temp - (m * l * theta_acc * cos_t) / (M + m)
        
        # Механічні відбійники рейки
        if x_val >= X_RAIL:
            f_bump = -K_BUMPER * (x_val - X_RAIL) - D_BUMPER * max(0.0, v_val)
            x_acc += f_bump / M
        elif x_val <= -X_RAIL:
            f_bump = -K_BUMPER * (x_val + X_RAIL) - D_BUMPER * min(0.0, v_val)
            x_acc += f_bump / M
            
        return [v_val, w_val, x_acc, theta_acc]

    sol = solve_ivp(dynamics, (0, SIM_DURATION), init_state, t_eval=t_eval, method='RK45')
    
    # Розрахунок масиву керуючого зусилля u для кожного моменту часу
    u_history = np.zeros_like(sol.t)
    for idx in range(len(sol.t)):
        x_val, th_val, v_val, w_val = sol.y[:, idx]
        th_wrapped = (th_val + np.pi) % (2 * np.pi) - np.pi
        if motor_enabled and abs(th_wrapped) < np.radians(70.0):
            u_calc = k_th * th_wrapped + k_om * w_val + k_pos * x_val + k_vel * v_val
            u_history[idx] = np.clip(u_calc, -U_MAX, U_MAX)
        else:
            u_history[idx] = 0.0
            
    return sol.t, sol.y, u_history


# =============================================================================
# 6. ГРАФІЧНИЙ ІНТЕРФЕЙС ТА АНІМАЦІЯ (MATPLOTLIB)
# =============================================================================
def run_simulation_gui():
    print_system_info()
    
    current_params = {
        'k_theta': k_theta_active,
        'k_omega': k_omega_active,
        'k_x': k_x_active,
        'k_v': k_v_active,
        'init_theta': INIT_THETA_DEG,
        'motor_on': True
    }
    
    t_data, state_data, u_data = simulate_trajectory(
        current_params['k_theta'],
        current_params['k_omega'],
        current_params['k_x'],
        current_params['k_v'],
        current_params['init_theta'],
        current_params['motor_on']
    )
    
    # Налаштування вікна
    fig = plt.figure(figsize=(13, 8))
    fig.canvas.manager.set_window_title("Симулятор стабілізації маятника (Cart-Pole)")
    
    # Сітка розташування: Зліва - анімація, Справа - графіки, Знизу - повзунки
    gs = fig.add_gridspec(2, 2, height_ratios=[1.7, 1.0], width_ratios=[1.3, 1.0])
    
    ax_anim = fig.add_subplot(gs[0, 0])
    ax_plots_top = fig.add_subplot(gs[0, 1])
    ax_plots_bot = fig.add_subplot(gs[1, 1])
    ax_controls = fig.add_subplot(gs[1, 0])
    ax_controls.axis('off')
    
    # -------------------------------------------------------------
    # Налаштування поля анімації
    # -------------------------------------------------------------
    ax_anim.set_xlim(-X_RAIL - 0.35, X_RAIL + 0.35)
    ax_anim.set_ylim(-L - 0.35, L + 0.35)
    ax_anim.set_aspect('equal')
    ax_anim.grid(True, linestyle=':', alpha=0.5)
    ax_anim.set_title("Фізична модель: Вагонетка та Маятник", fontsize=11, fontweight='bold')
    ax_anim.set_xlabel("Положення рейки (метри)")
    
    # Рейка
    ax_anim.plot([-X_RAIL - 0.3, X_RAIL + 0.3], [0, 0], 'k-', lw=3, zorder=1)
    
    # Відбійники на краях
    ax_anim.axvline(x=X_RAIL, color='crimson', linestyle='--', lw=2, label=f'Відбійник (+{X_RAIL}м)')
    ax_anim.axvline(x=-X_RAIL, color='crimson', linestyle='--', lw=2, label=f'Відбійник (-{X_RAIL}м)')
    
    # Візуальні об'єкти візка
    cart_w, cart_h = 0.36, 0.16
    cart_rect = plt.Rectangle((-cart_w/2, -cart_h/2), cart_w, cart_h,
                              facecolor='#2b5c8f', edgecolor='#133152', lw=2, zorder=3)
    ax_anim.add_patch(cart_rect)
    
    # Колеса візка
    wheel_l = plt.Circle((-cart_w/3, -cart_h/2), 0.045, facecolor='#333333', zorder=4)
    wheel_r = plt.Circle((cart_w/3, -cart_h/2), 0.045, facecolor='#333333', zorder=4)
    ax_anim.add_patch(wheel_l)
    ax_anim.add_patch(wheel_r)
    
    # Маятник: стержень та вантаж на кінці
    pend_line, = ax_anim.plot([], [], '-', color='#d9381e', lw=5, solid_capstyle='round', zorder=5)
    pend_bob = plt.Circle((0, 0), 0.07, facecolor='#b01e07', edgecolor='#400800', lw=1.5, zorder=6)
    ax_anim.add_patch(pend_bob)
    pivot_joint = plt.Circle((0, 0), 0.025, facecolor='#ffffff', edgecolor='#000000', lw=2, zorder=7)
    ax_anim.add_patch(pivot_joint)
    
    # Стрілка сили мотора
    force_arrow, = ax_anim.plot([], [], color='#00aa44', lw=3, zorder=8)
    
    # Телеметрія HUD
    hud_text = ax_anim.text(0.03, 0.95, "", transform=ax_anim.transAxes,
                            fontsize=9, verticalalignment='top',
                            bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8f9fa', alpha=0.9, edgecolor='#ccc'))
    
    # -------------------------------------------------------------
    # Налаштування графіків у реальному часі (Справа)
    # -------------------------------------------------------------
    # Графік 1: Кут theta(t)
    ax_plots_top.set_title("Кут маятника θ(t) [градуси]", fontsize=10, fontweight='bold')
    ax_plots_top.set_xlim(0, SIM_DURATION)
    ax_plots_top.set_ylim(-35, 35)
    ax_plots_top.grid(True, linestyle=':', alpha=0.6)
    ax_plots_top.axhline(0, color='gray', lw=1)
    ax_plots_top.axhspan(-3, 3, color='green', alpha=0.15, label='Цільова зона (±3°)')
    line_theta_plot, = ax_plots_top.plot([], [], color='#d9381e', lw=1.8, label='θ(t)')
    marker_theta, = ax_plots_top.plot([], [], 'o', color='#b01e07', ms=6)
    ax_plots_top.legend(loc='upper right', fontsize=8)
    
    # Графік 2: Позиція візка x(t)
    ax_plots_bot.set_title("Положення візка x(t) [метри]", fontsize=10, fontweight='bold')
    ax_plots_bot.set_xlim(0, SIM_DURATION)
    ax_plots_bot.set_ylim(-X_RAIL - 0.2, X_RAIL + 0.2)
    ax_plots_bot.grid(True, linestyle=':', alpha=0.6)
    ax_plots_bot.axhline(0, color='gray', lw=1)
    ax_plots_bot.axhline(X_RAIL, color='crimson', linestyle='--', lw=1.2)
    ax_plots_bot.axhline(-X_RAIL, color='crimson', linestyle='--', lw=1.2)
    line_x_plot, = ax_plots_bot.plot([], [], color='#2b5c8f', lw=1.8, label='x(t)')
    marker_x, = ax_plots_bot.plot([], [], 'o', color='#133152', ms=6)
    ax_plots_bot.set_xlabel("Час (секунди)")
    ax_plots_bot.legend(loc='upper right', fontsize=8)
    
    # -------------------------------------------------------------
    # Інтерактивні елементи керування (Повзунки та Кнопки)
    # -------------------------------------------------------------
    slider_w = 0.28
    btn_x = 0.46
    btn_w = 0.11
    
    # Створюємо позиції для повзунків
    ax_s_kth = fig.add_axes([0.11, 0.24, slider_w, 0.025])
    ax_s_kom = fig.add_axes([0.11, 0.19, slider_w, 0.025])
    ax_s_kx  = fig.add_axes([0.11, 0.14, slider_w, 0.025])
    ax_s_kv  = fig.add_axes([0.11, 0.09, slider_w, 0.025])
    ax_s_th0 = fig.add_axes([0.11, 0.04, slider_w, 0.025])
    
    slider_kth = Slider(ax_s_kth, 'k_θ (кут)', 0.0, 300.0, valinit=current_params['k_theta'], valfmt='%.1f')
    slider_kom = Slider(ax_s_kom, 'k_ω (демпфер)', 0.0, 100.0, valinit=current_params['k_omega'], valfmt='%.1f')
    slider_kx  = Slider(ax_s_kx,  'k_x (позиція)', 0.0, 60.0,  valinit=current_params['k_x'], valfmt='%.1f')
    slider_kv  = Slider(ax_s_kv,  'k_v (швидкість)', 0.0, 60.0, valinit=current_params['k_v'], valfmt='%.1f')
    slider_th0 = Slider(ax_s_th0, 'θ₀ (поч. нахил°)', -40.0, 40.0, valinit=current_params['init_theta'], valfmt='%.1f°')
    
    # Кнопки
    ax_btn_lqr   = fig.add_axes([btn_x, 0.21, btn_w, 0.045])
    ax_btn_motor = fig.add_axes([btn_x, 0.13, btn_w, 0.045])
    ax_btn_reset = fig.add_axes([btn_x, 0.05, btn_w, 0.045])
    
    btn_lqr   = Button(ax_btn_lqr, '★ Авто LQR', color='#d8ecf8', hovercolor='#beddf4')
    btn_motor = Button(ax_btn_motor, 'Мотор: УВІМК', color='#d5f5e3', hovercolor='#abebc6')
    btn_reset = Button(ax_btn_reset, '↻ Перезапуск', color='#fcf3cf', hovercolor='#f9e79f')
    
    # Оновлення графіків даними симуляції
    def refresh_plot_lines():
        theta_degs = np.degrees(state_data[1])
        x_vals = state_data[0]
        
        line_theta_plot.set_data(t_data, theta_degs)
        line_x_plot.set_data(t_data, x_vals)
        
        max_th = max(35, float(np.max(np.abs(theta_degs)) * 1.15))
        ax_plots_top.set_ylim(-max_th, max_th)
    
    refresh_plot_lines()
    
    # Функція перерахунку при зміні повзунків
    def update_simulation_data(reset_frame=True):
        nonlocal t_data, state_data, u_data, anim_frame_idx
        t_data, state_data, u_data = simulate_trajectory(
            current_params['k_theta'],
            current_params['k_omega'],
            current_params['k_x'],
            current_params['k_v'],
            current_params['init_theta'],
            current_params['motor_on']
        )
        refresh_plot_lines()
        if reset_frame:
            anim_frame_idx = 0
            
    def on_slider_change(val):
        current_params['k_theta']   = slider_kth.val
        current_params['k_omega']   = slider_kom.val
        current_params['k_x']       = slider_kx.val
        current_params['k_v']       = slider_kv.val
        current_params['init_theta'] = slider_th0.val
        update_simulation_data(reset_frame=True)
        
    slider_kth.on_changed(on_slider_change)
    slider_kom.on_changed(on_slider_change)
    slider_kx.on_changed(on_slider_change)
    slider_kv.on_changed(on_slider_change)
    slider_th0.on_changed(on_slider_change)
    
    def on_lqr_clicked(event):
        slider_kth.set_val(k_th_lqr)
        slider_kom.set_val(k_om_lqr)
        slider_kx.set_val(k_x_lqr)
        slider_kv.set_val(k_v_lqr)
        print(f"[LQR] Встановлено оптимальні константи: k_th={k_th_lqr:.2f}, k_om={k_om_lqr:.2f}, k_x={k_x_lqr:.2f}, k_v={k_v_lqr:.2f}")
        
    btn_lqr.on_clicked(on_lqr_clicked)
    
    def on_motor_toggle(event):
        current_params['motor_on'] = not current_params['motor_on']
        if current_params['motor_on']:
            btn_motor.label.set_text("Мотор: УВІМК")
            btn_motor.color = '#d5f5e3'
        else:
            btn_motor.label.set_text("Мотор: ВИМК")
            btn_motor.color = '#fadbd8'
        update_simulation_data(reset_frame=True)
        
    btn_motor.on_clicked(on_motor_toggle)
    
    def on_reset_clicked(event):
        update_simulation_data(reset_frame=True)
        
    btn_reset.on_clicked(on_reset_clicked)

    # -------------------------------------------------------------
    # Цикл анімації
    # -------------------------------------------------------------
    anim_frame_idx = 0
    total_frames = len(t_data)
    
    def animate_step(i):
        nonlocal anim_frame_idx
        idx = anim_frame_idx % total_frames
        anim_frame_idx += 1
        
        x_c   = state_data[0][idx]
        th    = state_data[1][idx]
        v_c   = state_data[2][idx]
        w_p   = state_data[3][idx]
        u_val = u_data[idx]
        t_val = t_data[idx]
        
        # Оновлення геометрії візка
        cart_rect.set_x(x_c - cart_w / 2.0)
        wheel_l.center = (x_c - cart_w / 3.0, -cart_h / 2.0)
        wheel_r.center = (x_c + cart_w / 3.0, -cart_h / 2.0)
        
        # Кінець маятника (стержень довжиною L)
        x_tip = x_c + L * np.sin(th)
        y_tip = L * np.cos(th)
        
        pend_line.set_data([x_c, x_tip], [0, y_tip])
        pend_bob.center = (x_tip, y_tip)
        pivot_joint.center = (x_c, 0)
        
        # Стрілка напрямку сили мотора
        arrow_len = (u_val / U_MAX) * 0.4
        if abs(arrow_len) > 0.02:
            force_arrow.set_data([x_c, x_c + arrow_len], [-cart_h * 0.8, -cart_h * 0.8])
            force_arrow.set_color('#27ae60' if arrow_len > 0 else '#e67e22')
        else:
            force_arrow.set_data([], [])
            
        # Маркери на графіках праворуч
        th_deg = np.degrees(th)
        marker_theta.set_data([t_val], [th_deg])
        marker_x.set_data([t_val], [x_c])
        
        # Статус стабілізації
        th_wrapped_deg = np.degrees((th + np.pi) % (2 * np.pi) - np.pi)
        if not current_params['motor_on']:
            status_str = "МОТОР ВИМКНЕНО (Вільне хитання)"
        elif abs(th_wrapped_deg) < 2.5 and abs(x_c) < 0.1:
            status_str = "СТАБІЛІЗОВАНО (Вертикаль утримана!)"
        elif abs(th_wrapped_deg) < 45.0:
            status_str = "В ПРОЦЕСІ СТАБІЛІЗАЦІЇ..."
        else:
            status_str = "ВПАВ (Кут занадто великий)"
            
        hud_text.set_text(
            f"Час: {t_val:4.2f} с | Статус: {status_str}\n"
            f"Кут θ: {th_deg:5.1f}° | Швидкість ω: {np.degrees(w_p):5.1f}°/с\n"
            f"Позиція x: {x_c:5.2f} м | Швидкість v: {v_c:5.2f} м/с\n"
            f"Сила мотора u: {u_val:5.1f} Н  (k_θ={current_params['k_theta']:.1f}, k_ω={current_params['k_omega']:.1f}, k_x={current_params['k_x']:.1f}, k_v={current_params['k_v']:.1f})"
        )
        
        return cart_rect, wheel_l, wheel_r, pend_line, pend_bob, pivot_joint, force_arrow, marker_theta, marker_x, hud_text

    global ani
    ani = animation.FuncAnimation(
        fig, animate_step,
        frames=total_frames, interval=int(1000 / FPS), blit=False
    )
    
    fig.subplots_adjust(bottom=0.32, top=0.93, left=0.08, right=0.96, hspace=0.38, wspace=0.25)
    plt.show()
    return ani


if __name__ == '__main__':
    anim_obj = run_simulation_gui()
