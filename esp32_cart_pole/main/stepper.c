#include "stepper.h"
#include "cart_pole_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "STEPPER";

#define STEP_PCNT_LIMIT 30000

static pcnt_unit_handle_t s_unit = NULL;
static uint32_t s_freq = 0;   // поточна частота кроків (0 = стоп)
static int s_dir = 1;         // 1 = вправо (+X), 0 = вліво
static bool s_running = false;
static double s_est_steps = 0.0; // розрахункові кроки (резерв/діагностика)
static double s_x_offset_m = 0.0;
static bool s_counter_ok = true;
static float s_check_t = 0.0f;
static double s_check_est0 = 0.0;
static int32_t s_check_cnt0 = 0;

static int32_t pcnt_steps(void) {
  int v = 0;
  if (s_unit)
    pcnt_unit_get_count(s_unit, &v);
  return (int32_t)v;
}

void stepper_init(void) {
  // DIR та EN — вихід + вхід (PCNT читає рівень DIR для знаку кроку)
  gpio_config_t io = {.pin_bit_mask = (1ULL << STEPPER_PIN_CW) | (1ULL << STEPPER_PIN_EN),
                      .mode = GPIO_MODE_INPUT_OUTPUT,
                      .pull_up_en = GPIO_PULLUP_DISABLE,
                      .pull_down_en = GPIO_PULLDOWN_DISABLE,
                      .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&io));
  gpio_set_level(STEPPER_PIN_EN, 0); // увімкнено
  gpio_set_level(STEPPER_PIN_CW, 1);
  s_dir = 1;

  // LEDC: явне джерело APB, щоб ledc_set_freq не перемикав джерело тактування
  ledc_timer_config_t tcfg = {.speed_mode = LEDC_PWM_MODE,
                              .duty_resolution = LEDC_PWM_RES,
                              .timer_num = LEDC_PWM_TIMER,
                              .freq_hz = 1000,
                              .clk_cfg = LEDC_USE_APB_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&tcfg));
  ledc_channel_config_t ccfg = {.gpio_num = STEPPER_PIN_CLK,
                                .speed_mode = LEDC_PWM_MODE,
                                .channel = LEDC_PWM_CH,
                                .timer_sel = LEDC_PWM_TIMER,
                                .duty = 0,
                                .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

  // PCNT: +1 на фронт CLK при DIR=1, -1 при DIR=0
  pcnt_unit_config_t ucfg = {.high_limit = STEP_PCNT_LIMIT,
                             .low_limit = -STEP_PCNT_LIMIT,
                             .flags.accum_count = true};
  ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &s_unit));
  pcnt_chan_config_t ch = {.edge_gpio_num = STEPPER_PIN_CLK, .level_gpio_num = STEPPER_PIN_CW};
  pcnt_channel_handle_t chan = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &ch, &chan));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                               PCNT_CHANNEL_EDGE_ACTION_HOLD));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  // Без watch point на межах накопичення переповнень НЕ працює
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, STEP_PCNT_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, -STEP_PCNT_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_enable(s_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(s_unit));
  // Гарантуємо, що вхідний тракт піна CLK увімкнено (вихід лишається за LEDC)
  gpio_input_enable(STEPPER_PIN_CLK);

  s_freq = 0;
  s_running = false;
  s_est_steps = 0.0;
  s_x_offset_m = 0.0;
}

void stepper_enable(bool enable) { gpio_set_level(STEPPER_PIN_EN, enable ? 0 : 1); }

void stepper_stop(void) {
  if (s_running) {
    ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 0);
    ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
    s_running = false;
  }
  s_freq = 0;
}

void stepper_set_velocity(float vel_mps) {
  float f = fabsf(vel_mps) * STEPS_PER_METER;
  if (f < (float)MIN_STEP_FREQ) {
    stepper_stop();
    return;
  }
  if (f > (float)MAX_STEP_FREQ_HZ)
    f = (float)MAX_STEP_FREQ_HZ;
  uint32_t freq = (uint32_t)lroundf(f);

  int dir = vel_mps > 0.0f ? 1 : 0;
  if (dir != s_dir) {
    gpio_set_level(STEPPER_PIN_CW, dir);
    s_dir = dir;
  }
  if (freq != s_freq) {
    // Дільник застосовується на наступному переповненні таймера — без збою фази
    if (ledc_set_freq(LEDC_PWM_MODE, LEDC_PWM_TIMER, freq) != ESP_OK)
      return;
    s_freq = freq;
  }
  if (!s_running) {
    ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 1u << (LEDC_PWM_RES - 1)); // 50%
    ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
    s_running = true;
  }
}

void stepper_tick(float dt) {
  if (s_running)
    s_est_steps += (s_dir ? 1.0 : -1.0) * (double)s_freq * dt;

  // Самоперевірка лічильника: за 0.5 с руху PCNT має побачити ≥ 50% кроків
  if (s_counter_ok) {
    s_check_t += dt;
    if (s_check_t >= 0.5f) {
      double est = s_est_steps - s_check_est0;
      int32_t cnt = pcnt_steps() - s_check_cnt0;
      if (fabs(est) > 200.0 && fabs((double)cnt) < 0.5 * fabs(est)) {
        s_counter_ok = false;
        // Перейти на розрахункову позицію, зберігши поточну X
        double x = (double)pcnt_steps() / STEPS_PER_METER + s_x_offset_m;
        s_x_offset_m = x - s_est_steps / STEPS_PER_METER;
        ESP_LOGE(TAG, "PCNT не бачить кроків (%ld з ~%.0f)! Позиція — з розрахунку.",
                 (long)cnt, est);
      }
      s_check_t = 0.0f;
      s_check_est0 = s_est_steps;
      s_check_cnt0 = pcnt_steps();
    }
  }
}

static double raw_steps(void) {
  return s_counter_ok ? (double)pcnt_steps() : s_est_steps;
}

float stepper_get_position_m(void) {
  return (float)(raw_steps() / STEPS_PER_METER + s_x_offset_m);
}

void stepper_set_position_m(float x_m) {
  s_x_offset_m = (double)x_m - raw_steps() / STEPS_PER_METER;
}

int32_t stepper_get_counted_steps(void) { return pcnt_steps(); }
int32_t stepper_get_estimated_steps(void) { return (int32_t)lround(s_est_steps); }
bool stepper_counter_ok(void) { return s_counter_ok; }
