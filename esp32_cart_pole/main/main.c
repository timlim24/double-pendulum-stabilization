#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

// Спільний конфігураційний файл стенду (геометрія рейки, піни, фізика
// LQR/Cascade)
#include "cart_pole_config.h"

static const char *TAG = "CART_POLE";

// =========================================================================
// ЗМІННІ СТАНУ
// =========================================================================
typedef enum {
  STATE_HOMING_SEEK_LEFT = 0, // Пошук лівого кінцевика (Нуль)
  STATE_HOMING_GO_CENTER,     // Виїзд у центр (RAIL_CENTER_M)
  STATE_RETURNING_TO_CENTER,  // Автоповернення візка в центр після падіння
  STATE_READY_FOR_BALANCE,    // Очікування спокійного підйому маятника вгору
  STATE_BALANCING,            // Активна стабілізація маятника
  STATE_SAFETY_STOP           // Аварійна зупинка
} system_state_t;

static volatile system_state_t sys_state = STATE_HOMING_SEEK_LEFT;
static pcnt_unit_handle_t pcnt_unit = NULL;

static volatile float cart_x =
    0.0f; // Координата від центру (м), 0.0 = центр рейки
static volatile float cart_v = 0.0f; // Швидкість візка (м/с)
static volatile float pendulum_theta =
    0.0f; // Відхилення від верхньої вертикалі (рад, 0 = строго вгору)
static volatile float pendulum_omega =
    0.0f; // Фільтрована кутова швидкість (рад/с)

// =========================================================================
// КЕРУВАННЯ ШВИДКІСТЮ (АПАРАТНИЙ LEDC З КЕШУВАННЯМ)
// =========================================================================
static uint32_t s_last_freq = 0;
static int s_last_dir = -1;

static void set_motor_velocity(float vel_mps) {
  if (vel_mps > MAX_CART_VELOCITY)
    vel_mps = MAX_CART_VELOCITY;
  if (vel_mps < -MAX_CART_VELOCITY)
    vel_mps = -MAX_CART_VELOCITY;

  // Апаратне блокування стоперами
  if (vel_mps < 0 && gpio_get_level(STOPPER_LEFT_PIN) == 0) {
    vel_mps = 0.0f;
  }
  if (vel_mps > 0 && gpio_get_level(STOPPER_RIGHT_PIN) == 0) {
    vel_mps = 0.0f;
  }

  float abs_vel = fabsf(vel_mps);
  uint32_t freq_hz = (uint32_t)(abs_vel * STEPS_PER_METER);

  if (freq_hz < MIN_STEP_FREQ) {
    cart_v = 0.0f;
    if (s_last_freq != 0) {
      ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 0);
      ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
      s_last_freq = 0;
    }
    return;
  }

  cart_v = vel_mps;
  int dir = (vel_mps >= 0) ? 1 : 0;

  if (dir != s_last_dir) {
    gpio_set_level(STEPPER_PIN_CW, dir);
    s_last_dir = dir;
  }

  if (freq_hz != s_last_freq) {
    ledc_set_freq(LEDC_PWM_MODE, LEDC_PWM_TIMER, freq_hz);
    ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 512);
    ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
    s_last_freq = freq_hz;
  }
}

// =========================================================================
// ПЕРЕРАХУНОК КУТА (ВЕРХНЯ ВЕРТИКАЛЬ = 0 РАДІАН)
// =========================================================================
// Маятник калібрується внизу (висить вільно під дією гравітації = 0 тіків)
// Відповідно верхня вертикаль знаходиться рівно на 180° (±1200 тіків = ±PI)
static inline float calculate_theta_from_raw(int raw_ticks) {
  float raw_rad = (float)raw_ticks * RAD_PER_TICK;

  // Нормалізуємо до [-PI, PI], де 0 = маятник висить униз
  float angle_from_bottom = fmodf(raw_rad, 2.0f * (float)M_PI);
  if (angle_from_bottom > (float)M_PI)
    angle_from_bottom -= 2.0f * (float)M_PI;
  if (angle_from_bottom < -(float)M_PI)
    angle_from_bottom += 2.0f * (float)M_PI;

  // Відхилення від ВЕРХНЬОЇ вертикалі (де верх = ±PI від низу)
  float theta_from_top = 0.0f;
  if (angle_from_bottom >= 0.0f) {
    theta_from_top = angle_from_bottom - (float)M_PI;
  } else {
    theta_from_top = angle_from_bottom + (float)M_PI;
  }
  return theta_from_top;
}

// =========================================================================
// ІНІЦІАЛІЗАЦІЯ ОБЛАДНАННЯ
// =========================================================================
static void init_hardware(void) {
  // 1. Стопери
  gpio_config_t io_stoppers = {.pin_bit_mask = (1ULL << STOPPER_LEFT_PIN) |
                                               (1ULL << STOPPER_RIGHT_PIN),
                               .mode = GPIO_MODE_INPUT,
                               .pull_up_en = GPIO_PULLUP_ENABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_stoppers);

  // 2. Сигнали TB6560 (тільки DIR і EN, пін CLK/PUL налаштує LEDC)
  gpio_config_t io_stepper = {.pin_bit_mask = (1ULL << STEPPER_PIN_CW) |
                                              (1ULL << STEPPER_PIN_EN),
                              .mode = GPIO_MODE_OUTPUT,
                              .pull_up_en = GPIO_PULLUP_DISABLE,
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_stepper);

  gpio_set_level(STEPPER_PIN_EN, 0); // 0 = Увімкнено (утримання валу)
  gpio_set_level(STEPPER_PIN_CW, 0);

  // 3. Апаратний LEDC на піні GPIO 18 (PUL+)
  ledc_timer_config_t timer_conf = {.speed_mode = LEDC_PWM_MODE,
                                    .duty_resolution = LEDC_PWM_RES,
                                    .timer_num = LEDC_PWM_TIMER,
                                    .freq_hz = 1000,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  ledc_channel_config_t ch_conf = {.gpio_num = STEPPER_PIN_CLK,
                                   .speed_mode = LEDC_PWM_MODE,
                                   .channel = LEDC_PWM_CH,
                                   .timer_sel = LEDC_PWM_TIMER,
                                   .duty = 0,
                                   .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

  // 4. Енкодер LPD3806
  pcnt_unit_config_t unit_config = {
      .high_limit = 32000,
      .low_limit = -32000,
      .flags.accum_count = true,
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = 1000,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

  pcnt_chan_config_t chan_a = {
      .edge_gpio_num = ENC_PIN_A,
      .level_gpio_num = ENC_PIN_B,
  };
  pcnt_channel_handle_t pcnt_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a, &pcnt_a));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(pcnt_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(pcnt_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  pcnt_chan_config_t chan_b = {
      .edge_gpio_num = ENC_PIN_B,
      .level_gpio_num = ENC_PIN_A,
  };
  pcnt_channel_handle_t pcnt_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b, &pcnt_b));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(pcnt_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(pcnt_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  gpio_set_pull_mode(ENC_PIN_A, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(ENC_PIN_B, GPIO_PULLUP_ONLY);

  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

// =========================================================================
// КОНТУР СТАБІЛІЗАЦІЇ КАСКАДНИЙ (100 ГЦ)
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

  ESP_LOGI(TAG, "=== СТАРТ КОНТУРУ: ПОШУК ЛІВОГО СТОПЕРА (НУЛЬ) ===");

  while (1) {
    vTaskDelayUntil(&last_wake, period);

    // Зчитування енкодера та перерахунок кута відносно верхньої вертикалі
    pcnt_unit_get_count(pcnt_unit, &raw_enc);
    pendulum_theta = calculate_theta_from_raw(raw_enc);

    // Обчислення кутової швидкості з захистом від стрибка фази wrap-around
    // (+-PI)
    float delta_theta = pendulum_theta - prev_theta;
    if (delta_theta > (float)M_PI)
      delta_theta -= 2.0f * (float)M_PI;
    if (delta_theta < -(float)M_PI)
      delta_theta += 2.0f * (float)M_PI;

    float raw_omega = delta_theta / dt;
    pendulum_omega = 0.65f * pendulum_omega + 0.35f * raw_omega;
    prev_theta = pendulum_theta;

    // Оновлення координати візка
    cart_x += cart_v * dt;

    // Аварійний захист під час активного балансування
    if (sys_state == STATE_BALANCING) {
      if (gpio_get_level(STOPPER_LEFT_PIN) == 0 ||
          gpio_get_level(STOPPER_RIGHT_PIN) == 0 ||
          fabsf(cart_x) > SOFT_LIMIT_X_M) {
        set_motor_velocity(0.0f);
        target_v = 0.0f;
        ESP_LOGE(
            TAG,
            "Аварія: спрацював стопер або досягнуто краю рейки (X = %.3fm)!",
            cart_x);
        sys_state = STATE_SAFETY_STOP;
      }
    }

    switch (sys_state) {
    case STATE_HOMING_SEEK_LEFT: {
      // Якщо лівий стопер натиснуто (GPIO32 == 0)
      if (gpio_get_level(STOPPER_LEFT_PIN) == 0) {
        set_motor_velocity(0.0f);
        target_v = 0.0f;
        cart_x =
            -RAIL_CENTER_M; // Лівий стопер це -RAIL_CENTER_M відносно центру
        ESP_LOGI(TAG,
                 "Лівий стопер знайдено! Виїжджаю в центр вільного ходу (%.2f "
                 "см / %lu кр)...",
                 RAIL_CENTER_M * 100.0f, RAIL_CENTER_STEPS);
        sys_state = STATE_HOMING_GO_CENTER;
      } else {
        set_motor_velocity(-HOMING_SPEED_MPS);
      }
      break;
    }

    case STATE_HOMING_GO_CENTER: {
      // Рух праворуч рівно до точки cart_x = 0.0 м
      if (cart_x >= 0.0f) {
        set_motor_velocity(0.0f);
        target_v = 0.0f;
        cart_x = 0.0f;
        // Обнуляємо енкодер, поки маятник спокійно висить унизу
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
        prev_theta = 0.0f;
        pendulum_omega = 0.0f;
        ESP_LOGI(TAG, "=== ВІЗОК У ЦЕНТРІ (%.2f см)! ГОТОВИЙ ДО БАЛАНСУ! ===",
                 RAIL_CENTER_M * 100.0f);
        ESP_LOGI(TAG,
                 "Підніміть маятник рукою до вертикалі (180°) і затримайте...");
        sys_state = STATE_READY_FOR_BALANCE;
      } else {
        set_motor_velocity(HOMING_SPEED_MPS);
      }
      break;
    }

    case STATE_READY_FOR_BALANCE: {
      set_motor_velocity(0.0f);
      target_v = 0.0f;

      // Очікуємо спокійного утримання маятника у верхній точці (< 3.5° та |w| <
      // 0.8 рад/с)
      if (fabsf(pendulum_theta) < BALANCE_CATCH_ANGLE &&
          fabsf(pendulum_omega) < BALANCE_CATCH_OMEGA) {
        target_v = 0.0f;
        ESP_LOGI(TAG,
                 "Маятник у зоні вертикалі (%.1f°, w=%.2f рад/с)! БАЛАНС "
                 "АКТИВОВАНО!",
                 pendulum_theta * 180.0f / (float)M_PI, pendulum_omega);
        sys_state = STATE_BALANCING;
      }
      break;
    }

    case STATE_BALANCING: {
      // Якщо маятник упав (> 20.0°)
      if (fabsf(pendulum_theta) > BALANCE_FALL_ANGLE) {
        set_motor_velocity(0.0f);
        target_v = 0.0f;
        ESP_LOGW(TAG, "Маятник упав. Повертаю візок назад у центр %.2f см...",
                 RAIL_CENTER_M * 100.0f);
        sys_state = STATE_RETURNING_TO_CENTER;
        break;
      }

      // 1. Зовнішній контур: цільовий нахил маятника для повернення візка до x
      // = 0
      float theta_target = (K_CART_P * cart_x + K_CART_D * cart_v);
      if (theta_target > MAX_TARGET_TILT_RAD)
        theta_target = MAX_TARGET_TILT_RAD;
      if (theta_target < -MAX_TARGET_TILT_RAD)
        theta_target = -MAX_TARGET_TILT_RAD;

      // Похибка кута відносно цільового нахилу
      float theta_err = pendulum_theta - theta_target;

      // 2. Внутрішній швидкий контур балансу: бажане прискорення візка
      // Знак мінус забезпечує від'ємний зворотний зв'язок (Negative Feedback)
      float accel_cmd = -(K_THETA * theta_err + K_OMEGA * pendulum_omega);

      // Плавне обмеження максимального прискорення мотора
      if (accel_cmd > MAX_CART_ACCEL)
        accel_cmd = MAX_CART_ACCEL;
      if (accel_cmd < -MAX_CART_ACCEL)
        accel_cmd = -MAX_CART_ACCEL;

      target_v += accel_cmd * dt;
      set_motor_velocity(target_v);
      break;
    }

    case STATE_RETURNING_TO_CENTER: {
      // Автоматичне плавне повернення в центр (cart_x = 0)
      float dist_to_center = -cart_x;

      if (fabsf(dist_to_center) <= 0.003f) {
        set_motor_velocity(0.0f);
        target_v = 0.0f;
        cart_x = 0.0f;
        prev_theta = pendulum_theta;
        pendulum_omega = 0.0f;
        ESP_LOGI(TAG, "Візок повернувся в центр. ГОТОВИЙ ДО БАЛАНСУ! Підніміть "
                      "маятник...");
        sys_state = STATE_READY_FOR_BALANCE;
      } else {
        float return_speed =
            (dist_to_center > 0) ? RECENTER_SPEED_MPS : -RECENTER_SPEED_MPS;
        set_motor_velocity(return_speed);
      }
      break;
    }

    case STATE_SAFETY_STOP: {
      set_motor_velocity(0.0f);
      target_v = 0.0f;
      break;
    }
    }
  }
}

// =========================================================================
// ТЕЛЕМЕТРІЯ (5 ГЦ)
// =========================================================================
static void telemetry_task(void *pvParameters) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(200));

    const char *st_str = "ОЧІКУВАННЯ";
    if (sys_state == STATE_BALANCING)
      st_str = "\033[32mАКТИВНИЙ БАЛАНС\033[0m";
    else if (sys_state == STATE_RETURNING_TO_CENTER)
      st_str = "\033[36mПОВЕРНЕННЯ В ЦЕНТР\033[0m";
    else if (sys_state == STATE_READY_FOR_BALANCE)
      st_str = "\033[33mВ ЦЕНТРІ (ПІДНІМІТЬ)\033[0m";
    else if (sys_state == STATE_HOMING_SEEK_LEFT)
      st_str = "ПОШУК НУЛЯ";
    else if (sys_state == STATE_HOMING_GO_CENTER)
      st_str = "ВИЇЗД У ЦЕНТР";
    else if (sys_state == STATE_SAFETY_STOP)
      st_str = "\033[31mАВАРІЙНИЙ СТОП\033[0m";

    int l_st = gpio_get_level(STOPPER_LEFT_PIN);
    int r_st = gpio_get_level(STOPPER_RIGHT_PIN);

    printf("[%s] X: %+5.3fm | V: %+5.2fm/s | L:%d R:%d | Кут: %+6.2f° | w: "
           "%+5.2f rad/s\r\n",
           st_str, cart_x, cart_v, l_st, r_st,
           pendulum_theta * 180.0f / (float)M_PI, pendulum_omega);
  }
}

void app_main(void) {
  ESP_LOGI(TAG,
           "Старт системи: Рейка = %.2f см, Візок = %.1f см, Робочий хід = "
           "%.2f см, Центр = %.2f см (100%% апаратний LEDC)...",
           RAIL_BETWEEN_STOPPERS_M * 100.0f, CART_WIDTH_M * 100.0f,
           RAIL_DISTANCE_M * 100.0f, RAIL_CENTER_M * 100.0f);
  init_hardware();

  xTaskCreatePinnedToCore(lqr_control_task, "lqr_task", 8192, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);
  xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 2, NULL, 0);
}
