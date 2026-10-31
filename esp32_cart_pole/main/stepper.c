#include "stepper.h"
#include "cart_pole_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "stoppers.h"
#include <math.h>

static uint32_t s_last_freq = 0;
static int s_last_dir = -1;
static float s_current_v = 0.0f;

void stepper_init(void) {
  // 1. Сигнали TB6560: DIR та EN
  gpio_config_t io_stepper = {.pin_bit_mask = (1ULL << STEPPER_PIN_CW) |
                                              (1ULL << STEPPER_PIN_EN),
                              .mode = GPIO_MODE_OUTPUT,
                              .pull_up_en = GPIO_PULLUP_DISABLE,
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_stepper);

  gpio_set_level(STEPPER_PIN_EN, 0); // 0 = Увімкнено (утримання валу)
  gpio_set_level(STEPPER_PIN_CW, 0);

  // 2. Апаратний LEDC на піні GPIO 18 (PUL+)
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

  s_last_freq = 0;
  s_last_dir = -1;
  s_current_v = 0.0f;
}

void stepper_enable(bool enable) {
  gpio_set_level(STEPPER_PIN_EN, enable ? 0 : 1);
}

void stepper_stop(void) {
  s_current_v = 0.0f;
  if (s_last_freq != 0) {
    ledc_set_duty(LEDC_PWM_MODE, LEDC_PWM_CH, 0);
    ledc_update_duty(LEDC_PWM_MODE, LEDC_PWM_CH);
    s_last_freq = 0;
  }
}

float stepper_get_velocity(void) { return s_current_v; }

void stepper_set_velocity(float vel_mps) {
  if (vel_mps > MAX_CART_VELOCITY) {
    vel_mps = MAX_CART_VELOCITY;
  }
  if (vel_mps < -MAX_CART_VELOCITY) {
    vel_mps = -MAX_CART_VELOCITY;
  }

  // Апаратне блокування стоперами (Active LOW): заборона руху далі в бік
  // натиснутого стопера
  if (vel_mps < 0.0f && stopper_left_pressed()) {
    vel_mps = 0.0f;
  }
  if (vel_mps > 0.0f && stopper_right_pressed()) {
    vel_mps = 0.0f;
  }

  float abs_vel = fabsf(vel_mps);
  uint32_t freq_hz = (uint32_t)(abs_vel * STEPS_PER_METER);

  if (freq_hz < MIN_STEP_FREQ) {
    stepper_stop();
    return;
  }

  s_current_v = vel_mps;
  // Полярність мотора на стенді: dir = 0 для руху вліво (v < 0), dir = 1 для
  // руху вправо (v > 0)
  int dir = (vel_mps >= 0.0f) ? 1 : 0;

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
