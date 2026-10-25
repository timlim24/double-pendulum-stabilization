#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

// Спільний конфігураційний файл стенду
#include "cart_pole_config.h"

static const char *TAG = "BENCH_TEST";

// Цільова швидкість для тесту: 10.0 см/с = 0.10 м/с -> 5000 Гц
#define TARGET_SPEED_MPS 0.10f
#define TARGET_FREQ_HZ                                                         \
  ((uint32_t)(TARGET_SPEED_MPS * STEPS_PER_METER)) // 5000 Гц

// Межі випадкових проміжків часу руху (секунди)
#define MIN_INTERVAL_SEC 0.5f
#define MAX_INTERVAL_SEC 5.0f

static pcnt_unit_handle_t pcnt_unit = NULL;

static float get_random_interval(float min_s, float max_s) {
  uint32_t r = esp_random();
  return min_s + ((float)r / (float)UINT32_MAX) * (max_s - min_s);
}

static void init_hardware(void) {
  // 1. Стопери
  gpio_config_t io_conf_stoppers = {.pin_bit_mask = (1ULL << STOPPER_LEFT_PIN) |
                                                    (1ULL << STOPPER_RIGHT_PIN),
                                    .mode = GPIO_MODE_INPUT,
                                    .pull_up_en = GPIO_PULLUP_ENABLE,
                                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                    .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf_stoppers);

  // 2. Драйвер TB6560
  gpio_config_t io_conf_stepper = {.pin_bit_mask = (1ULL << STEPPER_PIN_CW) |
                                                   (1ULL << STEPPER_PIN_EN),
                                   .mode = GPIO_MODE_OUTPUT,
                                   .pull_up_en = GPIO_PULLUP_DISABLE,
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf_stepper);

  gpio_set_level(STEPPER_PIN_EN, 0); // 0 = Увімкнено
  gpio_set_level(STEPPER_PIN_CW, 0);

  // 3. Апаратний генератор кроків LEDC (ідеально рівна частота без мікрофризів)
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

  // 4. Енкодер PCNT
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

  // Канал A
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

  // Канал B
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

// Зупинка генератора кроків
static void stop_stepper(void) {
  ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 0);
  ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
}

// Ідеально плавний рух з апаратним розгоном та гальмуванням (без дрижання)
static bool smooth_move_duration(float duration_sec, int dir) {
  gpio_set_level(STEPPER_PIN_CW, dir);
  vTaskDelay(pdMS_TO_TICKS(5));

  // 1. Плавний апаратний розгін (Ramp Up за 80 мс від 800 Гц до 5000 Гц)
  const uint32_t start_freq = 800;
  const uint32_t target_freq = TARGET_FREQ_HZ;
  const int ramp_steps = 8;

  for (int i = 1; i <= ramp_steps; i++) {
    // Перевірка стоперів
    if (dir == 1 && gpio_get_level(STOPPER_RIGHT_PIN) == 0) {
      stop_stepper();
      return false;
    }
    if (dir == 0 && gpio_get_level(STOPPER_LEFT_PIN) == 0) {
      stop_stepper();
      return false;
    }

    uint32_t f = start_freq + ((target_freq - start_freq) * i) / ramp_steps;
    ledc_set_freq(LEDC_PWM_MODE, LEDC_PWM_TIMER, f);
    ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 512); // 50%
    ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // 2. Рух на постійній стабільній швидкості (10 см/с)
  int64_t start_time = esp_timer_get_time();
  int64_t duration_us = (int64_t)(duration_sec * 1000000.0f);

  while ((esp_timer_get_time() - start_time) < duration_us) {
    // Перевірка кінцевиків кожні 10 мс
    if (dir == 1 && gpio_get_level(STOPPER_RIGHT_PIN) == 0) {
      ESP_LOGW(TAG, "!!! СТОП: СПРАЦЮВАВ ПРАВИЙ СТОПЕР (GPIO 33 = 0) !!!");
      stop_stepper();
      return false;
    }
    if (dir == 0 && gpio_get_level(STOPPER_LEFT_PIN) == 0) {
      ESP_LOGW(TAG, "!!! СТОП: СПРАЦЮВАВ ЛІВИЙ СТОПЕР (GPIO 32 = 0) !!!");
      stop_stepper();
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(
        10)); // FreeRTOS спокійно спить, поки залізо LEDC генерує кроки
  }

  // 3. Плавне гальмування (Ramp Down за 80 мс)
  for (int i = ramp_steps - 1; i >= 1; i--) {
    uint32_t f = start_freq + ((target_freq - start_freq) * i) / ramp_steps;
    ledc_set_freq(LEDC_PWM_MODE, LEDC_PWM_TIMER, f);
    ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stop_stepper();
  return true;
}

static void main_test_loop(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(300));
  int cycle = 0;
  int enc_count = 0;

  while (1) {
    cycle++;

    // 1. Сенсори
    int left_stop = gpio_get_level(STOPPER_LEFT_PIN);
    int right_stop = gpio_get_level(STOPPER_RIGHT_PIN);
    int enc_a = gpio_get_level(ENC_PIN_A);
    int enc_b = gpio_get_level(ENC_PIN_B);
    pcnt_unit_get_count(pcnt_unit, &enc_count);
    float angle_deg = (float)enc_count * DEG_PER_TICK;

    printf(
        "\n==================== [ ТЕСТОВИЙ ЦИКЛ #%d ] ====================\n",
        cycle);
    printf("СТОПЕРИ:  Лівий(D32)=%d [%s]  |  Правий(D33)=%d [%s]\n", left_stop,
           (left_stop == 0 ? "НАТИСНУТО" : "ВІЛЬНО"), right_stop,
           (right_stop == 0 ? "НАТИСНУТО" : "ВІЛЬНО"));
    printf(
        "ЕНКОДЕР:  Піни: A(D16)=%d, B(D17)=%d  |  Тіки: %d  |  Кут: %.2f deg\n",
        enc_a, enc_b, enc_count, angle_deg);
    printf("==============================================================\n");

    // 2. Рух ВПЕРЕД
    if (gpio_get_level(STOPPER_RIGHT_PIN) != 0) {
      float forward_duration =
          get_random_interval(MIN_INTERVAL_SEC, MAX_INTERVAL_SEC);
      printf(">> Мотор: Апаратний плавний розгін ВПЕРЕД (%.1f с, швидкість: 10 "
             "см/с)...\n",
             forward_duration);
      smooth_move_duration(forward_duration, 1);
    } else {
      printf(">> Правий стопер затиснутий! Пропуск руху вперед.\n");
    }

    // Швидка пауза 200 мс
    vTaskDelay(pdMS_TO_TICKS(200));

    // 3. Рух НАЗАД
    if (gpio_get_level(STOPPER_LEFT_PIN) != 0) {
      float backward_duration =
          get_random_interval(MIN_INTERVAL_SEC, MAX_INTERVAL_SEC);
      printf("<< Мотор: Апаратний плавний розгін НАЗАД (%.1f с, швидкість: 10 "
             "см/с)...\n",
             backward_duration);
      smooth_move_duration(backward_duration, 0);
    } else {
      printf("<< Лівий стопер затиснутий! Пропуск руху назад.\n");
    }

    // Швидка пауза 200 мс
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Ініціалізація апаратного тесту стенду Cart-Pole...");
  init_hardware();

  xTaskCreatePinnedToCore(main_test_loop, "main_test_loop", 4096, NULL, 5, NULL,
                          1);
}
