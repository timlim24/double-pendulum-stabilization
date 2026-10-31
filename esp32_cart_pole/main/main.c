#include "cart_pole_config.h"
#include "encoder.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpc_trajectory_data.h"
#include "stepper.h"
#include "stoppers.h"
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

static const char *TAG = "CART_POLE";

// =========================================================================
// АВТОМАТ СТАНІВ СИСТЕМИ
// =========================================================================
typedef enum {
  STATE_HOMING_SEEK_LEFT = 0, // Пошук нульової точки рейки
  STATE_HOMING_GO_CENTER = 1, // Переміщення в точний центр рейки (cart_x = 0)
  STATE_ACTIVE_ZEROING = 2,   // Активне гасіння коливань маятника внизу
  STATE_WAITING_FOR_TRIGGER = 3, // Очікування команди [s] з ноутбука
  STATE_SWINGUP = 4,             // Оптимальний підйом за траєкторією MPC
  STATE_BALANCING = 5,           // LQR стабілізація у верхній точці
  STATE_RETURNING_TO_CENTER = 6, // Плавне повернення в центр після падіння
  STATE_SAFETY_STOP = 7,         // Аварійний стоп
  STATE_SYSID = 8 // Автономна багатоетапна системна ідентифікація
} system_state_t;

static volatile system_state_t sys_state = STATE_HOMING_SEEK_LEFT;
static volatile bool s_trigger_swingup_requested = false;
static volatile bool s_trigger_zero_requested = false;
static volatile bool s_trigger_home_requested = false;
static volatile bool s_trigger_sysid_requested = false;

// Стан системи
static volatile float cart_x = 0.0f; // Координата від центру рейки (м)
static volatile float cart_v = 0.0f; // Лінійна швидкість візка (м/с)
static volatile float pendulum_theta =
    0.0f; // Відхилення від верхньої вертикалі (рад, 0 = строго вгору)
static volatile float pendulum_omega =
    0.0f; // Фільтрована кутова швидкість (рад/с)

// =========================================================================
// ПРЯМЕ ГАСІННЯ КІНЕТИЧНОЇ ЕНЕРГІЇ ВНИЗУ (DIRECT ENERGY DAMPING)
// =========================================================================
static inline void step_active_downward_stabilization(float dt,
                                                      float angle_from_bottom,
                                                      float omega,
                                                      float *target_v) {
  // Пряме демпфування швидкості: v_desired = -k_damp * omega - k_x * cart_x
  float v_desired = -DOWN_K_OMEGA_VEL * omega - DOWN_K_X_VEL * cart_x;

  if (v_desired > DOWN_MAX_VELOCITY)
    v_desired = DOWN_MAX_VELOCITY;
  if (v_desired < -DOWN_MAX_VELOCITY)
    v_desired = -DOWN_MAX_VELOCITY;

  // Безпечна зона рейки (не далі ±15 см під час гасіння)
  if (cart_x <= -DOWN_SAFE_LIMIT_X_M && v_desired < 0.0f)
    v_desired = 0.0f;
  if (cart_x >= DOWN_SAFE_LIMIT_X_M && v_desired > 0.0f)
    v_desired = 0.0f;

  if ((stopper_left_pressed() || stopper_left_raw()) && v_desired < 0.0f)
    v_desired = 0.0f;
  if ((stopper_right_pressed() || stopper_right_raw()) && v_desired > 0.0f)
    v_desired = 0.0f;

  // Плавне наростання швидкості
  float max_dv = MAX_CART_ACCEL * dt;
  float dv = v_desired - *target_v;
  if (dv > max_dv)
    dv = max_dv;
  if (dv < -max_dv)
    dv = -max_dv;

  *target_v += dv;
  stepper_set_velocity(*target_v);
}

// =========================================================================
// КОНТУР КЕРУВАННЯ ТА СТАБІЛІЗАЦІЇ (100 ГЦ)
// =========================================================================
static void lqr_control_task(void *pvParameters) {
  TickType_t period = pdMS_TO_TICKS(10);
  if (period == 0)
    period = 1;

  TickType_t last_wake = xTaskGetTickCount();
  const float dt = 0.010f; // 100 Гц контур керування

  int raw_enc = 0;
  float prev_theta = 0.0f;
  float target_v = 0.0f;
  float cart_x_integral = 0.0f;
  int settle_counter = 0;
  float swing_playback_time = 0.0f;

  // Змінні для автономного SysID
  int sysid_trial = 1; // 1, 2, 3
  int sysid_substate =
      0; // 0 = рух/збудження, 1 = активне заспокоєння перед наступним
  float sysid_trial_time = 0.0f;
  int sysid_damp_counter = 0;

  ESP_LOGI(TAG, "=== СТАРТ КОНТУРУ СТАБІЛІЗАЦІЇ (100 Гц) ===");

  while (1) {
    vTaskDelayUntil(&last_wake, period);

    // 0. Оновлення цифрового фільтра та дебаунсу кінцевиків
    stoppers_update();

    // 1. Безперервне зчитування енкодера та перерахунок кутів (ніколи не
    // зупиняється)
    raw_enc = encoder_get_raw_count();
    pendulum_theta = encoder_calculate_theta(raw_enc);
    float angle_from_bottom = encoder_calculate_bottom_angle(raw_enc);

    // 2. Обчислення кутової швидкості з захистом від стрибка фази (±PI)
    float delta_theta = pendulum_theta - prev_theta;
    if (delta_theta > (float)M_PI)
      delta_theta -= 2.0f * (float)M_PI;
    if (delta_theta < -(float)M_PI)
      delta_theta += 2.0f * (float)M_PI;

    float raw_omega = delta_theta / dt;
    pendulum_omega = 0.65f * pendulum_omega + 0.35f * raw_omega;
    prev_theta = pendulum_theta;

    // 3. Оновлення координати та швидкості візка
    cart_v = stepper_get_velocity();
    cart_x += cart_v * dt;

    // 4. Фізична калібрація положення при контакті зі стоперами
    // Якщо торкнулися стопера — візок точно на краю! Відкат у центр!
    if (stopper_left_pressed() || stopper_left_raw()) {
      cart_x = -RAIL_CENTER_M;
      if (target_v < 0.0f)
        target_v = 0.0f;
      if (sys_state == STATE_WAITING_FOR_TRIGGER ||
          sys_state == STATE_ACTIVE_ZEROING) {
        ESP_LOGW(TAG, "Контакт з лівим стопером! Повернення у центр...");
        sys_state = STATE_RETURNING_TO_CENTER;
      }
    } else if (stopper_right_pressed() || stopper_right_raw()) {
      cart_x = RAIL_CENTER_M;
      if (target_v > 0.0f)
        target_v = 0.0f;
      if (sys_state == STATE_WAITING_FOR_TRIGGER ||
          sys_state == STATE_ACTIVE_ZEROING) {
        ESP_LOGW(TAG, "Контакт з правим стопером! Повернення у центр...");
        sys_state = STATE_RETURNING_TO_CENTER;
      }
    }

    // Обробка ручних команд з консолі
    if (s_trigger_zero_requested) {
      s_trigger_zero_requested = false;
      stepper_stop();
      target_v = 0.0f;
      cart_v = 0.0f;
      cart_x = 0.0f;
      cart_x_integral = 0.0f;
      encoder_reset();
      prev_theta = pendulum_theta;
      pendulum_omega = 0.0f;
      settle_counter = 0;
      ESP_LOGI(TAG, "\033[36m>>> РУЧНЕ ВСТАНОВЛЕННЯ ЦЕНТРУ (X=0). Очікую "
                    "команду [s]... <<<\033[0m");
      sys_state = STATE_WAITING_FOR_TRIGGER;
    }

    if (s_trigger_home_requested) {
      s_trigger_home_requested = false;
      stepper_stop();
      target_v = 0.0f;
      cart_x = 0.0f;
      ESP_LOGI(
          TAG,
          "\033[33m>>> Перезапуск пошуку кінцевика (Homing)... <<<\033[0m");
      sys_state = STATE_HOMING_SEEK_LEFT;
    }

    // =========================================================================
    // ГЛОБАЛЬНЕ ПЕРЕХОПЛЕННЯ У ВЕРХНІЙ ТОЧЦІ (OMNIPRESENT LQR BALANCE CATCH)
    // Якщо маятник опиняється біля верхньої вертикалі — ЗАВЖДИ вмикаємо LQR!
    // Миттєво перериває будь-який інший стан (гасіння, розгін, очікування,
    // повернення).
    // =========================================================================
    if (sys_state != STATE_HOMING_SEEK_LEFT &&
        sys_state != STATE_HOMING_GO_CENTER && sys_state != STATE_SAFETY_STOP &&
        sys_state != STATE_BALANCING && sys_state != STATE_SYSID) {
      if (fabsf(pendulum_theta) < GLOBAL_CATCH_ANGLE &&
          fabsf(pendulum_omega) < GLOBAL_CATCH_OMEGA &&
          !(stopper_left_pressed() && cart_x < -0.18f) &&
          !(stopper_right_pressed() && cart_x > 0.18f)) {
        ESP_LOGI(TAG,
                 "\033[32m★ ВЕРХНЮ ТОЧКУ ЗЛОВЛЕНО! АКТИВАЦІЯ LQR БАЛАНСУ! "
                 "(Кут=%+.1f°, w=%+.2f рад/с, X=%+.3fm, стан %d)\033[0m",
                 pendulum_theta * 180.0f / (float)M_PI, pendulum_omega, cart_x,
                 (int)sys_state);
        target_v = cart_v;
        cart_x_integral = 0.0f;
        sys_state = STATE_BALANCING;
      }
    }

    // 5. Автомат станів
    switch (sys_state) {
    case STATE_HOMING_SEEK_LEFT: {
      if (stopper_left_pressed() || stopper_left_raw()) {
        stepper_stop();
        target_v = 0.0f;
        cart_x = -RAIL_CENTER_M;
        ESP_LOGI(
            TAG,
            "Лівий стопер знайдено! Виїжджаю в центр (%.2f см / %lu кр)...",
            RAIL_CENTER_M * 100.0f, (unsigned long)RAIL_CENTER_STEPS);
        sys_state = STATE_HOMING_GO_CENTER;
      } else if (stopper_right_pressed() || stopper_right_raw()) {
        stepper_stop();
        target_v = 0.0f;
        cart_x = RAIL_CENTER_M;
        ESP_LOGI(TAG,
                 "Правий стопер зафіксовано! Виїжджаю в центр (%.2f см)...",
                 RAIL_CENTER_M * 100.0f);
        sys_state = STATE_HOMING_GO_CENTER;
      } else if (fabsf(cart_x) > 0.65f) {
        stepper_stop();
        target_v = 0.0f;
        cart_x = 0.0f;
        encoder_reset();
        prev_theta = pendulum_theta;
        pendulum_omega = 0.0f;
        settle_counter = 0;
        ESP_LOGW(TAG, "УВАГА: Кінцевики не спрацювали за 65 см! Приймаю "
                      "поточну позицію за X=0.");
        sys_state = STATE_ACTIVE_ZEROING;
      } else {
        stepper_set_velocity(-HOMING_SPEED_MPS);
      }
      break;
    }

    case STATE_HOMING_GO_CENTER: {
      float dist_to_center = -cart_x;
      if (fabsf(dist_to_center) <= 0.005f) {
        stepper_stop();
        target_v = 0.0f;
        cart_x = 0.0f;
        cart_x_integral = 0.0f;
        encoder_reset();
        prev_theta = pendulum_theta;
        pendulum_omega = 0.0f;
        settle_counter = 0;
        ESP_LOGI(
            TAG,
            "=== ВІЗОК У ЦЕНТРІ (%.2f см). Заспокоєння коливань внизу... ===",
            RAIL_CENTER_M * 100.0f);
        sys_state = STATE_ACTIVE_ZEROING;
      } else {
        float speed =
            (dist_to_center > 0.0f) ? HOMING_SPEED_MPS : -HOMING_SPEED_MPS;
        stepper_set_velocity(speed);
      }
      break;
    }

    case STATE_RETURNING_TO_CENTER: {
      float dist_to_center = -cart_x;

      if (fabsf(dist_to_center) <= 0.005f) {
        stepper_stop();
        target_v = 0.0f;
        cart_x = 0.0f;
        cart_x_integral = 0.0f;
        settle_counter = 0;
        ESP_LOGI(TAG, "Візок повернувся в центр. Гасіння коливань внизу...");
        sys_state = STATE_ACTIVE_ZEROING;
      } else {
        float return_speed =
            (dist_to_center > 0.0f) ? RECENTER_SPEED_MPS : -RECENTER_SPEED_MPS;
        stepper_set_velocity(return_speed);
      }
      break;
    }

    case STATE_ACTIVE_ZEROING: {
      // Якщо під час гасіння контакт зі стопером — відкотитися у центр
      if (((stopper_left_pressed() || stopper_left_raw()) && cart_x < -0.15f) ||
          ((stopper_right_pressed() || stopper_right_raw()) &&
           cart_x > 0.15f)) {
        stepper_stop();
        target_v = 0.0f;
        settle_counter = 0;
        ESP_LOGW(TAG, "Стопер під час гасіння! Відкат у центр...");
        sys_state = STATE_RETURNING_TO_CENTER;
        break;
      }

      // Якщо користувач натиснув [s] і маятник уже майже спокійний
      if (s_trigger_swingup_requested && fabsf(angle_from_bottom) < 0.08f &&
          fabsf(pendulum_omega) < 0.20f) {
        s_trigger_swingup_requested = false;
        stepper_stop();
        target_v = 0.0f;
        cart_v = 0.0f;
        cart_x = 0.0f;
        cart_x_integral = 0.0f;
        swing_playback_time = 0.0f;
        ESP_LOGI(TAG, "\033[35m▶ СТАРТ MPC SWING-UP (натиснуто [s])...\033[0m");
        sys_state = STATE_SWINGUP;
        break;
      }
      if (s_trigger_sysid_requested) {
        s_trigger_sysid_requested = false;
        stepper_stop();
        target_v = 0.0f;
        cart_v = 0.0f;
        cart_x = 0.0f;
        sysid_trial = 1;
        sysid_substate = 0;
        sysid_trial_time = 0.0f;
        sysid_damp_counter = 0;
        ESP_LOGI(TAG, "[36m=== СТАРТ АВТОНОМНОГО БАГАТОЕТАПНОГО SysID ===[0m");
        sys_state = STATE_SYSID;
        break;
      }

      step_active_downward_stabilization(dt, angle_from_bottom, pendulum_omega,
                                         &target_v);

      // Критерій дійсно ідеального спокою
      bool is_truly_settled = (fabsf(angle_from_bottom) < 0.043f && // < 2.5°
                               fabsf(pendulum_omega) < 0.12f && // < 0.12 рад/с
                               fabsf(cart_x) < 0.015f &&        // < 15 мм
                               fabsf(cart_v) < 0.02f); // нерухомий візок

      if (is_truly_settled) {
        settle_counter++;
      } else {
        settle_counter = 0;
      }

      // Потрібно 120 циклів (1.20 секунди) безперервного спокою
      if (settle_counter >= 120) {
        stepper_stop();
        target_v = 0.0f;
        cart_v = 0.0f;
        cart_x = 0.0f;
        settle_counter = 0;
        encoder_reset(); // Фіксуємо точний нижній нуль програмним офсетом
        ESP_LOGI(TAG, "\033[33m=== МАЯТНИК СПОКІЙНИЙ ВНИЗУ (1.2с спокою). "
                      "Очікую команду [s] для старту Swing-Up ===\033[0m");
        sys_state = STATE_WAITING_FOR_TRIGGER;
      }
      break;
    }

    case STATE_WAITING_FOR_TRIGGER: {
      // Якщо візок знесло до стопера або далеко від центру — повернути!
      if ((stopper_left_pressed() || stopper_left_raw()) ||
          (stopper_right_pressed() || stopper_right_raw()) ||
          fabsf(cart_x) > 0.03f) {
        sys_state = STATE_RETURNING_TO_CENTER;
        break;
      }

      // М'яке утримання візка в точці X = 0
      float hold_v = -2.5f * cart_x;
      if (hold_v > 0.10f)
        hold_v = 0.10f;
      if (hold_v < -0.10f)
        hold_v = -0.10f;
      stepper_set_velocity(hold_v);

      // Безперервний моніторинг: якщо маятник збурили внизу (> 2.5° або швидко
      // рухається)
      if (fabsf(angle_from_bottom) > 0.045f || fabsf(pendulum_omega) > 0.20f) {
        settle_counter = 0;
        ESP_LOGI(TAG,
                 "Порушення спокою внизу (кут=%+.1f°, w=%+.2f)! Повернення до "
                 "активного гасіння...",
                 angle_from_bottom * 180.0f / (float)M_PI, pendulum_omega);
        sys_state = STATE_ACTIVE_ZEROING;
        break;
      }

      // Отримано команду запуску підйому з ноутбука!
      if (s_trigger_swingup_requested) {
        s_trigger_swingup_requested = false;
        stepper_stop();
        target_v = 0.0f;
        cart_v = 0.0f;
        cart_x = 0.0f;
        cart_x_integral = 0.0f;
        swing_playback_time = 0.0f;
        ESP_LOGI(TAG,
                 "\033[35m▶ СТАРТ MPC SWING-UP (%.2fс, межа ±19см)...\033[0m",
                 MPC_TRAJ_TOTAL_TIME);
        sys_state = STATE_SWINGUP;
        break;
      }

      // Отримано команду запуску автономного SysID
      if (s_trigger_sysid_requested) {
        s_trigger_sysid_requested = false;
        stepper_stop();
        target_v = 0.0f;
        cart_v = 0.0f;
        cart_x = 0.0f;
        sysid_trial = 1;
        sysid_substate = 0;
        sysid_trial_time = 0.0f;
        sysid_damp_counter = 0;
        ESP_LOGI(
            TAG,
            "\033[36m=== СТАРТ АВТОНОМНОГО БАГАТОЕТАПНОГО SysID ===\033[0m");
        sys_state = STATE_SYSID;
        break;
      }
      break;
    }

    // =========================================================================
    // АВТОНОМНИЙ БАГАТОЕТАПНИЙ SysID (3 АВТОНОМНИХ ЕКСПЕРИМЕНТИ БЕЗ РУК)
    // =========================================================================
    case STATE_SYSID: {
      sysid_trial_time += dt;
      float a_cmd = 0.0f;
      float v_cmd = 0.0f;

      // Підстан 0: Виконання автономного збудження та запис трейсу
      if (sysid_substate == 0) {
        if (sysid_trial == 1) {
          // Експеримент 1: Імпульсний розгін та вільне згасання (тривалість 6.0
          // с) t = 0..0.12 с: ривок вправо з прискоренням +3.0 м/с² (dx ~ 2.2
          // см) t = 0.12..0.24 с: гальмування з -3.0 м/с² до зупинки (x ~ 4.4
          // см) t = 0.24..0.80 с: плавний відкат у центр X = 0 t = 0.80..6.0 с:
          // нерухомий візок X = 0, маятник вільно коливається
          if (sysid_trial_time < 0.12f) {
            a_cmd = 3.0f;
            v_cmd = a_cmd * sysid_trial_time;
          } else if (sysid_trial_time < 0.24f) {
            a_cmd = -3.0f;
            v_cmd = 0.36f - 3.0f * (sysid_trial_time - 0.12f);
          } else if (sysid_trial_time < 0.80f) {
            a_cmd = -0.5f;
            v_cmd = -1.8f * cart_x;
          } else {
            a_cmd = 0.0f;
            v_cmd = -3.0f * cart_x; // тримати центр X = 0
          }

          if (sysid_trial_time >= 6.0f) {
            sysid_substate = 1; // перейти до заспокоєння перед тестом 2
            sysid_damp_counter = 0;
          }

        } else if (sysid_trial == 2) {
          // Експеримент 2: Неперервне гармонічне збудження (синус 1.15 Гц,
          // тривалість 6.5 с) x(t) = A * sin(omega * t), A = ±3.5 см (абсолютно
          // безпечно для рейки)
          float freq_hz = 1.15f;
          float omega_sw = 2.0f * (float)M_PI * freq_hz;
          float amp = 0.035f;

          if (sysid_trial_time < 6.5f) {
            v_cmd = amp * omega_sw * cosf(omega_sw * sysid_trial_time);
            a_cmd =
                -amp * omega_sw * omega_sw * sinf(omega_sw * sysid_trial_time);
          } else {
            sysid_substate = 1; // перейти до заспокоєння перед тестом 3
            sysid_damp_counter = 0;
          }

        } else if (sysid_trial == 3) {
          // Експеримент 3: Двополярний імпульс (поштовх вліво, потім вправо,
          // тривалість 6.0 с)
          if (sysid_trial_time < 0.12f) {
            a_cmd = -3.0f;
            v_cmd = -3.0f * sysid_trial_time;
          } else if (sysid_trial_time < 0.24f) {
            a_cmd = 3.0f;
            v_cmd = -0.36f + 3.0f * (sysid_trial_time - 0.12f);
          } else if (sysid_trial_time < 0.40f) {
            a_cmd = 3.0f;
            v_cmd = 3.0f * (sysid_trial_time - 0.24f);
          } else if (sysid_trial_time < 0.52f) {
            a_cmd = -3.0f;
            v_cmd = 0.48f - 3.0f * (sysid_trial_time - 0.40f);
          } else {
            a_cmd = 0.0f;
            v_cmd = -3.0f * cart_x; // тримати X = 0
          }

          if (sysid_trial_time >= 6.0f) {
            sysid_substate = 1;
            sysid_damp_counter = 0;
          }
        }

        stepper_set_velocity(v_cmd);
        // Стрімінг точних даних 100 Гц у порт (angle_from_bottom: 0 = низ)
        printf("SYSID:%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", sysid_trial,
               sysid_trial_time, cart_x, cart_v, a_cmd, angle_from_bottom,
               pendulum_omega);

      } else {
        // Підстан 1: Активне демпфування маятника до повного нуля перед наступним тестом
        step_active_downward_stabilization(dt, angle_from_bottom,
                                           pendulum_omega, &target_v);
        stepper_set_velocity(target_v);

        bool settled =
            (fabsf(angle_from_bottom) < 0.050f &&
             fabsf(pendulum_omega) < 0.15f && fabsf(cart_x) < 0.025f);
        if (settled) {
          sysid_damp_counter++;
        } else {
          sysid_damp_counter = 0;
        }

        // 50 циклів (0.5 секунди) спокою АБО 3.5 секунди максимального демпфування
        if (sysid_damp_counter >= 50 || sysid_trial_time >= 9.5f) {
          stepper_stop();
          target_v = 0.0f;
          cart_x = 0.0f;
          encoder_reset();
          sysid_trial++;

          if (sysid_trial > 3) {
            // Всі 3 експерименти успішно записані!
            printf("SYSID_ALL_DONE\r\n");
            ESP_LOGI(TAG,
                     "=== ВСІ 3 ЕКСПЕРИМЕНТИ SysID ЗАВЕРШЕНО УСПІШНО! ===");
            sys_state = STATE_ACTIVE_ZEROING;
            break;
          } else {
            // Перехід до наступного експерименту
            sysid_substate = 0;
            sysid_trial_time = 0.0f;
            sysid_damp_counter = 0;
            printf("SYSID_NEXT_TRIAL:%d\r\n", sysid_trial);
          }
        }
      }
      break;
    }

    case STATE_SWINGUP: {
      // А) Захист від удару об стопери біля краю
      if (((stopper_left_pressed() || stopper_left_raw()) && cart_x < -0.17f) ||
          ((stopper_right_pressed() || stopper_right_raw()) &&
           cart_x > 0.17f)) {
        stepper_stop();
        target_v = 0.0f;
        ESP_LOGW(TAG,
                 "Контакт зі стопером під час Swing-Up (X=%.3fm)! Повернення в "
                 "центр...",
                 cart_x);
        sys_state = STATE_RETURNING_TO_CENTER;
        break;
      }

      // Б) Якщо траєкторія MPC завершилася
      if (swing_playback_time >= MPC_TRAJ_TOTAL_TIME) {
        ESP_LOGW(TAG, "MPC траєкторія завершилась. Повернення в центр...");
        sys_state = STATE_RETURNING_TO_CENTER;
        break;
      }

      // В) Отримання опорної точки з оптимальної траєкторії CasADi / IPOPT
      int idx = (int)(swing_playback_time / MPC_TRAJ_DT);
      if (idx >= MPC_TRAJ_POINTS - 1)
        idx = MPC_TRAJ_POINTS - 2;

      float frac =
          (swing_playback_time - (float)idx * MPC_TRAJ_DT) / MPC_TRAJ_DT;
      if (frac < 0.0f)
        frac = 0.0f;
      if (frac > 1.0f)
        frac = 1.0f;

      float x_ref = (1.0f - frac) * s_mpc_trajectory[idx].pos_m +
                    frac * s_mpc_trajectory[idx + 1].pos_m;
      float v_ref = (1.0f - frac) * s_mpc_trajectory[idx].vel_mps +
                    frac * s_mpc_trajectory[idx + 1].vel_mps;

      // Г) Відстеження траєкторії: Feedforward швидкості v_ref + корекція
      // похибки позиції
      const float K_POS_TRACK = 3.5f;
      float v_cmd = v_ref + K_POS_TRACK * (x_ref - cart_x);

      if (v_cmd > MAX_CART_VELOCITY)
        v_cmd = MAX_CART_VELOCITY;
      if (v_cmd < -MAX_CART_VELOCITY)
        v_cmd = -MAX_CART_VELOCITY;

      stepper_set_velocity(v_cmd);
      swing_playback_time += dt;
      break;
    }

    case STATE_BALANCING: {
      if (((stopper_left_pressed() || stopper_left_raw()) && cart_x < -0.17f) ||
          ((stopper_right_pressed() || stopper_right_raw()) &&
           cart_x > 0.17f)) {
        stepper_stop();
        target_v = 0.0f;
        ESP_LOGW(TAG,
                 "Контакт зі стопером під час балансу! Повернення в центр...");
        sys_state = STATE_RETURNING_TO_CENTER;
        break;
      }

      // Перевірка зриву балансу (гістерезис: зрив тільки при > 40.0°,
      // підхоплення при < 28.0°)
      if (fabsf(pendulum_theta) > BALANCE_FALL_ANGLE) {
        stepper_stop();
        target_v = 0.0f;
        ESP_LOGW(
            TAG,
            "Маятник упав (%.1f° > %.1f°)! Повернення візка в центр рейки...",
            pendulum_theta * 180.0f / (float)M_PI,
            BALANCE_FALL_ANGLE * 180.0f / (float)M_PI);
        sys_state = STATE_RETURNING_TO_CENTER;
        break;
      }

      // Каскадне керування балансом
      cart_x_integral += cart_x * dt;
      if (cart_x_integral > 0.5f)
        cart_x_integral = 0.5f;
      if (cart_x_integral < -0.5f)
        cart_x_integral = -0.5f;

      float theta_target =
          (K_CART_P * cart_x + K_CART_D * cart_v + K_CART_I * cart_x_integral);
      if (theta_target > MAX_TARGET_TILT_RAD)
        theta_target = MAX_TARGET_TILT_RAD;
      if (theta_target < -MAX_TARGET_TILT_RAD)
        theta_target = -MAX_TARGET_TILT_RAD;

      float theta_err = pendulum_theta - theta_target;
      float accel_cmd = -(K_THETA * theta_err + K_OMEGA * pendulum_omega);

      if (accel_cmd > MAX_CART_ACCEL)
        accel_cmd = MAX_CART_ACCEL;
      if (accel_cmd < -MAX_CART_ACCEL)
        accel_cmd = -MAX_CART_ACCEL;

      target_v += accel_cmd * dt;
      if (target_v > BALANCE_MAX_VELOCITY)
        target_v = BALANCE_MAX_VELOCITY;
      if (target_v < -BALANCE_MAX_VELOCITY)
        target_v = -BALANCE_MAX_VELOCITY;

      stepper_set_velocity(target_v);
      break;
    }

    case STATE_SAFETY_STOP: {
      stepper_stop();
      target_v = 0.0f;
      break;
    }
    }
  }
}

// =========================================================================
// ТЕЛЕМЕТРІЯ ТА ЧИТАННЯ КОМАНД З НОУТБУКА (10 ГЦ)
// =========================================================================
static void telemetry_task(void *pvParameters) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(100));

    // 1. Неблокуюче зчитування символів з USB UART (stdin)
    char c = 0;
    while (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == 's' || c == 'S') {
        s_trigger_swingup_requested = true;
      } else if (c == 'c' || c == 'C' || c == '0') {
        s_trigger_zero_requested = true;
      } else if (c == 'h' || c == 'H') {
        s_trigger_home_requested = true;
      } else if (c == 'i' || c == 'I' || c == 'e' || c == 'E') {
        s_trigger_sysid_requested = true;
      }
    }

    // Під час SysID стандартна телеметрія призупиняється, транслюється чистий
    // трейс SYSID
    if (sys_state == STATE_SYSID) {
      continue;
    }

    // 2. Вивід стандартної телеметрії з строго фіксованою шириною колонок
    const char *st_str = "ОЧІКУВАННЯ";
    if (sys_state == STATE_BALANCING)
      st_str = "АКТИВНИЙ БАЛАНС";
    else if (sys_state == STATE_SWINGUP)
      st_str = "MPC SWING-UP";
    else if (sys_state == STATE_WAITING_FOR_TRIGGER)
      st_str = "ОЧІКУЄ КОМАНДИ 's'";
    else if (sys_state == STATE_ACTIVE_ZEROING)
      st_str = "ЗАНУЛЕННЯ ВНИЗУ";
    else if (sys_state == STATE_RETURNING_TO_CENTER)
      st_str = "ПОВЕРНЕННЯ В ЦЕНТР";
    else if (sys_state == STATE_HOMING_SEEK_LEFT)
      st_str = "ПОШУК НУЛЯ";
    else if (sys_state == STATE_HOMING_GO_CENTER)
      st_str = "ВИЇЗД У ЦЕНТР";
    else if (sys_state == STATE_SAFETY_STOP)
      st_str = "АВАРІЙНИЙ СТОП";

    int l_st = stopper_left_pressed() ? 0 : 1;
    int r_st = stopper_right_pressed() ? 0 : 1;
    int enc_a = gpio_get_level(ENC_PIN_A);
    int enc_b = gpio_get_level(ENC_PIN_B);
    int enc_hw = encoder_get_raw_hardware_count();
    float deg_top = pendulum_theta * 180.0f / (float)M_PI;
    float deg_bot = deg_top + ((deg_top >= 0.0f) ? -180.0f : 180.0f);

    printf("[%s] X:%+6.3fm | V:%+5.2fm/s | L:%d R:%d | Кут:%+6.1f° (вниз:%+5.1f°) | Тіки:%+5d (A:%d B:%d) | w:%+5.2f\r\n",
           st_str, cart_x, cart_v, l_st, r_st, deg_top, deg_bot,
           enc_hw, enc_a, enc_b, pendulum_omega);
  }
}

void app_main(void) {
  ESP_LOGI(TAG,
           "Старт системи: Рейка = %.2f см, Візок = %.1f см, Робочий хід = "
           "%.2f см, Центр = %.2f см (Autonomous SysID + Debounced Stoppers + "
           "MPC)...",
           RAIL_BETWEEN_STOPPERS_M * 100.0f, CART_WIDTH_M * 100.0f,
           RAIL_DISTANCE_M * 100.0f, RAIL_CENTER_M * 100.0f);

  // Налаштування неблокуючого зчитування stdin
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

  // Ініціалізація периферійних модулів
  stoppers_init();
  stepper_init();
  encoder_init();

  xTaskCreatePinnedToCore(lqr_control_task, "lqr_task", 8192, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);
  xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 2, NULL, 0);
}
