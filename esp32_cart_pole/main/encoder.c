#include "encoder.h"
#include "cart_pole_config.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include <math.h>

static pcnt_unit_handle_t s_pcnt_unit = NULL;
static volatile int s_zero_offset = 0;

void encoder_init(void) {
  // Налаштування входів GPIO 16 та GPIO 17 з внутрішньою підтяжкою PULLUP
  // (критично для NPN Open-Collector енкодера LPD3806)
  gpio_config_t io_enc = {
      .pin_bit_mask = (1ULL << ENC_PIN_A) | (1ULL << ENC_PIN_B),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_enc);

  pcnt_unit_config_t unit_config = {
      .high_limit = 32000,
      .low_limit = -32000,
      .flags.accum_count = true,
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &s_pcnt_unit));

  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = 1000,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter_config));

  // Канал A
  pcnt_chan_config_t chan_a = {
      .edge_gpio_num = ENC_PIN_A,
      .level_gpio_num = ENC_PIN_B,
  };
  pcnt_channel_handle_t pcnt_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &chan_a, &pcnt_a));
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
  ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &chan_b, &pcnt_b));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(pcnt_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(pcnt_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_unit));
  s_zero_offset = 0;
}

void encoder_reset(void) {
  int count = 0;
  if (s_pcnt_unit != NULL) {
    pcnt_unit_get_count(s_pcnt_unit, &count);
  }
  s_zero_offset = count;
}

int encoder_get_raw_hardware_count(void) {
  int count = 0;
  if (s_pcnt_unit != NULL) {
    pcnt_unit_get_count(s_pcnt_unit, &count);
  }
  return count;
}

int encoder_get_raw_count(void) {
  int count = 0;
  if (s_pcnt_unit != NULL) {
    pcnt_unit_get_count(s_pcnt_unit, &count);
  }
  return count - s_zero_offset;
}

float encoder_calculate_bottom_angle(int raw_ticks) {
  float raw_rad = (float)raw_ticks * RAD_PER_TICK;
  float angle_from_bottom = fmodf(raw_rad, 2.0f * (float)M_PI);
  if (angle_from_bottom > (float)M_PI)
    angle_from_bottom -= 2.0f * (float)M_PI;
  if (angle_from_bottom < -(float)M_PI)
    angle_from_bottom += 2.0f * (float)M_PI;
  return angle_from_bottom;
}

float encoder_calculate_theta(int raw_ticks) {
  float angle_from_bottom = encoder_calculate_bottom_angle(raw_ticks);

  // Відхилення від ВЕРХНЬОЇ вертикалі (де верх = ±PI від низу)
  float theta_from_top = 0.0f;
  if (angle_from_bottom >= 0.0f) {
    theta_from_top = angle_from_bottom - (float)M_PI;
  } else {
    theta_from_top = angle_from_bottom + (float)M_PI;
  }
  return theta_from_top;
}

float encoder_get_theta(void) {
  return encoder_calculate_theta(encoder_get_raw_count());
}
