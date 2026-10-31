#include "stoppers.h"
#include "cart_pole_config.h"
#include "driver/gpio.h"

static int s_left_low_count = 0;
static int s_right_low_count = 0;

// Потрібно 4 послідовні цикли (40 мс при 100 Гц), щоб відсіяти ЕМЗ від мотора
#define STOPPER_DEBOUNCE_TICKS 4

void stoppers_init(void) {
  gpio_config_t io_stoppers = {.pin_bit_mask = (1ULL << STOPPER_LEFT_PIN) |
                                               (1ULL << STOPPER_RIGHT_PIN),
                               .mode = GPIO_MODE_INPUT,
                               .pull_up_en = GPIO_PULLUP_ENABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_stoppers);
  s_left_low_count = 0;
  s_right_low_count = 0;
}

void stoppers_update(void) {
  if (gpio_get_level(STOPPER_LEFT_PIN) == 0) {
    if (s_left_low_count < 100)
      s_left_low_count++;
  } else {
    s_left_low_count = 0;
  }

  if (gpio_get_level(STOPPER_RIGHT_PIN) == 0) {
    if (s_right_low_count < 100)
      s_right_low_count++;
  } else {
    s_right_low_count = 0;
  }
}

bool stopper_left_pressed(void) {
  return (s_left_low_count >= STOPPER_DEBOUNCE_TICKS);
}

bool stopper_right_pressed(void) {
  return (s_right_low_count >= STOPPER_DEBOUNCE_TICKS);
}

bool stopper_left_raw(void) { return (gpio_get_level(STOPPER_LEFT_PIN) == 0); }

bool stopper_right_raw(void) {
  return (gpio_get_level(STOPPER_RIGHT_PIN) == 0);
}
