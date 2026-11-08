#include "encoder.h"
#include "cart_pole_config.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

#define ENC_PCNT_LIMIT 30000

static pcnt_unit_handle_t s_unit = NULL;

void encoder_init(void) {
  // NPN open-collector: потрібна підтяжка (внутрішня слабка, краще 2.2–4.7 кОм зовні)
  gpio_config_t io = {
      .pin_bit_mask = (1ULL << ENC_PIN_A) | (1ULL << ENC_PIN_B),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io));

  pcnt_unit_config_t ucfg = {
      .high_limit = ENC_PCNT_LIMIT,
      .low_limit = -ENC_PCNT_LIMIT,
      .flags.accum_count = true,
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &s_unit));

  pcnt_glitch_filter_config_t filt = {.max_glitch_ns = ENC_GLITCH_NS};
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit, &filt));

  pcnt_chan_config_t ca = {.edge_gpio_num = ENC_PIN_A, .level_gpio_num = ENC_PIN_B};
  pcnt_channel_handle_t cha = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &ca, &cha));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                               PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  pcnt_chan_config_t cb = {.edge_gpio_num = ENC_PIN_B, .level_gpio_num = ENC_PIN_A};
  pcnt_channel_handle_t chb = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &cb, &chb));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                               PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // Watch point на межах — інакше переповнення губиться (accum_count не працює)
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, ENC_PCNT_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, -ENC_PCNT_LIMIT));

  ESP_ERROR_CHECK(pcnt_unit_enable(s_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(s_unit));
}

int32_t encoder_get_count(void) {
  int v = 0;
  if (s_unit)
    pcnt_unit_get_count(s_unit, &v);
  return (int32_t)v;
}
