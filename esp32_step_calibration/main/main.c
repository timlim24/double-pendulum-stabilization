#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Спільний конфігураційний файл стенду (геометрія рейки, піни, константи)
#include "cart_pole_config.h"

static const char *TAG = "CALIBRATION";

// Робочі частоти для калібрування
#define CALIB_SPEED_HZ 2500 // ~5.0 см/с
#define SLOW_TOUCH_HZ 800   // ~1.6 см/с для точного доторкання

// =========================================================================
// КЕРУВАННЯ ГЕНЕРАТОРОМ LEDC
// =========================================================================
static void stop_stepper(void) {
  ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 0);
  ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
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

  // 2. Сигнали TB6560 (DIR та ENA, пін CLK/PUL налаштовує виключно LEDC)
  gpio_config_t io_stepper = {.pin_bit_mask = (1ULL << STEPPER_PIN_CW) |
                                              (1ULL << STEPPER_PIN_EN),
                              .mode = GPIO_MODE_OUTPUT,
                              .pull_up_en = GPIO_PULLUP_DISABLE,
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_stepper);

  gpio_set_level(STEPPER_PIN_EN, 0); // 0 = Увімкнено (утримання валу)
  gpio_set_level(STEPPER_PIN_CW, 0);

  // 3. Апаратний генератор кроків LEDC на GPIO 18 (PUL+)
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
}

// =========================================================================
// РУХ ДО СТОПЕРА З ТОЧНИМ ВИМІРЮВАННЯМ ЧАСУ ТА КРОКІВ
// =========================================================================
static double move_until_stopper(int dir, int target_stopper_pin,
                                 uint32_t speed_hz, const char *label) {
  gpio_set_level(STEPPER_PIN_CW, dir);
  vTaskDelay(pdMS_TO_TICKS(30));

  const uint32_t start_freq = 800;
  const int ramp_steps = 10;
  double ramp_steps_accum = 0.0;

  int64_t t_start = esp_timer_get_time();

  // 1. Плавний апаратний розгін (100 мс)
  for (int i = 1; i <= ramp_steps; i++) {
    if (gpio_get_level(target_stopper_pin) == 0) {
      stop_stepper();
      return ramp_steps_accum;
    }
    uint32_t f = start_freq + ((speed_hz - start_freq) * i) / ramp_steps;
    ramp_steps_accum += (double)f * 0.010; // 10 мс
    ledc_set_freq(LEDC_PWM_MODE, LEDC_PWM_TIMER, f);
    ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 512); // 50%
    ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // 2. Рух на стабільній крейсерській частоті
  int64_t t_cruise_start = esp_timer_get_time();
  int update_count = 0;

  while (gpio_get_level(target_stopper_pin) != 0) {
    vTaskDelay(pdMS_TO_TICKS(10));
    update_count++;
    if ((update_count % 40) == 0) {
      int64_t now = esp_timer_get_time();
      double cruise_s = (double)(now - t_cruise_start) / 1000000.0;
      double total_s = (double)(now - t_start) / 1000000.0;
      double cur_steps = ramp_steps_accum + (cruise_s * (double)speed_hz);
      printf("[%s] Час: %4.1fc | Пройдено кроків: %6.0f | Частота: %lu Гц\r\n",
             label, total_s, cur_steps, speed_hz);
    }
  }

  // 3. Миттєва зупинка
  int64_t t_stop = esp_timer_get_time();
  stop_stepper();

  double cruise_duration_s = (double)(t_stop - t_cruise_start) / 1000000.0;
  double total_steps =
      ramp_steps_accum + (cruise_duration_s * (double)speed_hz);

  printf(">> Кінцевик спрацював! Час: %.3fс. Всього кроків: %.1f\r\n",
         (double)(t_stop - t_start) / 1000000.0, total_steps);

  return total_steps;
}

// =========================================================================
// РУХ НА ТОЧНУ КІЛЬКІСТЬ КРОКІВ
// =========================================================================
static void move_exact_steps(int dir, double target_steps, uint32_t speed_hz) {
  if (target_steps <= 0)
    return;

  gpio_set_level(STEPPER_PIN_CW, dir);
  vTaskDelay(pdMS_TO_TICKS(30));

  int64_t duration_us =
      (int64_t)((target_steps / (double)speed_hz) * 1000000.0);
  int64_t t0 = esp_timer_get_time();

  ledc_set_freq(LEDC_PWM_MODE, LEDC_PWM_TIMER, speed_hz);
  ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 512);
  ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);

  while ((esp_timer_get_time() - t0) < duration_us) {
    if (dir == 1 && gpio_get_level(STOPPER_RIGHT_PIN) == 0)
      break;
    if (dir == 0 && gpio_get_level(STOPPER_LEFT_PIN) == 0)
      break;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stop_stepper();
}

// =========================================================================
// ПРЕЦИЗІЙНЕ ДОТИКАННЯ (ДВОЕТАПНИЙ HOMING)
// =========================================================================
static void precision_home_left(void) {
  ESP_LOGI(TAG, "Швидкий виїзд до лівого стопера...");
  move_until_stopper(0, STOPPER_LEFT_PIN, CALIB_SPEED_HZ,
                     "ПОШУК ЛІВОГО СТОПЕРА");
  vTaskDelay(pdMS_TO_TICKS(300));

  // Від'їзд праворуч на ~1.5 см (~750 кроків)
  ESP_LOGI(TAG, "Прецизійний відкат назад на 750 кроків...");
  move_exact_steps(1, 750, 1500);
  vTaskDelay(pdMS_TO_TICKS(300));

  // Повільне мікро-наближення (800 Гц) для знаходження ідеального нуля
  ESP_LOGI(TAG, "Мікро-доторкання до нуля на малій швидкості...");
  move_until_stopper(0, STOPPER_LEFT_PIN, SLOW_TOUCH_HZ, "ТОЧНЕ ДОТИКАННЯ");
  vTaskDelay(pdMS_TO_TICKS(500));
  ESP_LOGI(TAG, ">>> Лівий нуль зафіксовано з абсолютною точністю! <<<");
}

// =========================================================================
// ГОЛОВНА ЗАДАЧА КАЛІБРУВАННЯ
// =========================================================================
static void calibration_task(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(1000));

  printf("\r\n");
  printf("==================================================================="
         "\r\n");
  printf(
      "     АВТОМАТИЧНИЙ ВИМІРЮВАЧ ФІЗИЧНОГО КРОКУ МОТОРА CART-POLE      \r\n");
  printf("==================================================================="
         "\r\n");
  printf(" Відстань між стоперами   : %.2f см (%.4f м)\r\n",
         RAIL_BETWEEN_STOPPERS_M * 100.0f, RAIL_BETWEEN_STOPPERS_M);
  printf(" Ширина візка             : %.2f см (%.4f м)\r\n",
         CART_WIDTH_M * 100.0f, CART_WIDTH_M);
  printf(" ФАКТИЧНИЙ РОБОЧИЙ ХІД    : \033[36m%.2f см\033[0m (%.1f мм / %.4f "
         "м)\r\n",
         RAIL_DISTANCE_CM, RAIL_DISTANCE_MM, RAIL_DISTANCE_M);
  printf(" Центр ходу від лівого нуля: \033[33m%.2f см\033[0m (%.4f м)\r\n",
         RAIL_CENTER_M * 100.0f, RAIL_CENTER_M);
  printf(" Робоча частота тесту     : %d Гц (~5.0 см/с)\r\n", CALIB_SPEED_HZ);
  printf(" Починаю перевірку кінцевиків...\r\n\r\n");

  int l_st = gpio_get_level(STOPPER_LEFT_PIN);
  int r_st = gpio_get_level(STOPPER_RIGHT_PIN);
  printf(
      "Стан стоперів зараз: Лівий(GPIO32)=%d [%s], Правий(GPIO33)=%d [%s]\r\n",
      l_st, (l_st == 0 ? "НАТИСНУТО" : "ВІЛЬНО"), r_st,
      (r_st == 0 ? "НАТИСНУТО" : "ВІЛЬНО"));

  if (r_st == 0 && l_st == 0) {
    ESP_LOGE(TAG, "Помилка: обидва стопери натиснуті або закорочені! Перевірте "
                  "проводку.");
    vTaskDelete(NULL);
    return;
  }

  // 1. Початкове прецизійне базування на лівий стопер (Нуль)
  precision_home_left();

  // 2. ПРОЇЗД 1: Зліва направо (від 0 до правого стопера)
  printf("\r\n-----------------------------------------------------------------"
         "--\r\n");
  printf(">> [ПРОЇЗД 1]: Рух ВПРАВО (Лівий стопер -> Правий стопер)...\r\n");
  double steps_forward =
      move_until_stopper(1, STOPPER_RIGHT_PIN, CALIB_SPEED_HZ, "ПРОЇЗД ВПРАВО");
  vTaskDelay(pdMS_TO_TICKS(800));

  // 3. ПРОЇЗД 2: Справа наліво (від правого стопера до лівого для верифікації)
  printf("\r\n-----------------------------------------------------------------"
         "--\r\n");
  printf(">> [ПРОЇЗД 2]: Зворотний рух ВЛІВО (Правий стопер -> Лівий "
         "стопер)...\r\n");
  double steps_backward =
      move_until_stopper(0, STOPPER_LEFT_PIN, CALIB_SPEED_HZ, "ПРОЇЗД ВЛІВО");
  vTaskDelay(pdMS_TO_TICKS(800));

  // 4. АНАЛІЗ РЕЗУЛЬТАТІВ ТА ФІЗИЧНИЙ РОЗРАХУНОК
  double diff = fabs(steps_forward - steps_backward);
  double avg_steps = (steps_forward + steps_backward) / 2.0;

  // Розрахунок ведеться від фактичного вільного ходу RAIL_DISTANCE (58.9 см)
  double steps_per_cm = avg_steps / (double)RAIL_DISTANCE_CM;
  double steps_per_mm = avg_steps / (double)RAIL_DISTANCE_MM;
  double steps_per_m = avg_steps / (double)RAIL_DISTANCE_M;
  double microns_per_step =
      (avg_steps > 0) ? ((double)RAIL_DISTANCE_MM / avg_steps) * 1000.0 : 0.0;
  double center_steps = avg_steps / 2.0;

  printf("\r\n================================================================="
         "==\r\n");
  printf(
      "                 РЕЗУЛЬТАТИ ТОЧНОГО КАЛІБРУВАННЯ                  \r\n");
  printf("==================================================================="
         "\r\n");
  printf(" Кроків Вперед (L -> R): \033[33m%.0f\033[0m кроків\r\n",
         steps_forward);
  printf(" Кроків Назад  (R -> L): \033[33m%.0f\033[0m кроків\r\n",
         steps_backward);
  printf(" Різниця (люфт/похибка): %.1f кроків (%.3f%%)\r\n", diff,
         (avg_steps > 0 ? (diff / avg_steps) * 100.0 : 0.0));

  if (diff <= (avg_steps * 0.005)) {
    printf(" \033[32m[ВІДМІННО]\033[0m: Ідеальна повторюваність, нуль "
           "пропущених кроків!\r\n");
  } else {
    printf(" \033[33m[УВАГА]\033[0m: Помітна різниця ходу. Перевірте натяг "
           "паса або струм драйвера.\r\n");
  }

  printf("-------------------------------------------------------------------"
         "\r\n");
  printf(" СЕРЕДНЯ КІЛЬКІСТЬ КРОКІВ НА РОБОЧИЙ ХІД (%.1f см): "
         "\033[32m%.1f\033[0m кроків\r\n",
         RAIL_DISTANCE_CM, avg_steps);
  printf("-------------------------------------------------------------------"
         "\r\n");
  printf(" ТОЧНІ ФІЗИЧНІ КОЕФІЦІЄНТИ ВАШОГО СТЕНДУ:\r\n");
  printf("  -> Кроків на 1 сантиметр : \033[36m%.2f\033[0m кроків/см\r\n",
         steps_per_cm);
  printf("  -> Кроків на 1 міліметр  : \033[36m%.3f\033[0m кроків/мм\r\n",
         steps_per_mm);
  printf("  -> Кроків на 1 метр      : \033[36m%.1f\033[0m кроків/м\r\n",
         steps_per_m);
  printf("  -> Дискретність 1 кроку  : \033[35m%.2f\033[0m мікронів (мкм)\r\n",
         microns_per_step);
  printf(
      "  -> Відстань до центру    : \033[33m%.0f\033[0m кроків (%.2f см)\r\n",
      center_steps, RAIL_CENTER_M * 100.0f);
  printf("==================================================================="
         "\r\n\r\n");

  printf("ОНОВЛЕННЯ ДЛЯ common/cart_pole_config.h:\r\n");
  printf("```c\r\n");
  printf("#define STEPS_PER_METER       %.1ff\r\n", steps_per_m);
  printf("```\r\n\r\n");

  // 5. Виїзд у точний математичний центр рейки
  printf(">> Виїжджаю рівно в центр рейки (%.0f кроків / %.2f см)...\r\n",
         center_steps, RAIL_CENTER_M * 100.0f);
  move_exact_steps(1, center_steps, 2000);
  printf(">> \033[32mВІЗОК СТОЇТЬ РІВНО В ЦЕНТРІ РЕЙКИ (%.2f см)!\033[0m Вал "
         "мотора заблоковано.\r\n",
         RAIL_CENTER_M * 100.0f);

  // 6. Періодичний статус
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("[СТЕНД В ЦЕНТРІ] Кроків/см: %.1f | Центр: %.0f кр (%.2f см) | L:%d "
           "R:%d\r\n",
           steps_per_cm, center_steps, RAIL_CENTER_M * 100.0f,
           gpio_get_level(STOPPER_LEFT_PIN), gpio_get_level(STOPPER_RIGHT_PIN));
  }
}

void app_main(void) {
  // Вимикаємо Task Watchdog, щоб вимірювальні проїзди не переривалися
  esp_task_wdt_deinit();

  ESP_LOGI(TAG, "Старт системи автоматичного калібрування кроку (100%% "
                "апаратний LEDC)...");
  init_hardware();

  xTaskCreatePinnedToCore(calibration_task, "calib_task", 8192, NULL, 5, NULL,
                          1);
}
