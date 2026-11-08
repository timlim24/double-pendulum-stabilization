#include "stoppers.h"
#include "cart_pole_config.h"
#include "driver/gpio.h"

static int s_left_cnt = 0;
static int s_right_cnt = 0;
static bool s_left = false;
static bool s_right = false;

void stoppers_init(void) {
  gpio_config_t io = {.pin_bit_mask = (1ULL << STOPPER_LEFT_PIN) | (1ULL << STOPPER_RIGHT_PIN),
                      .mode = GPIO_MODE_INPUT,
                      .pull_up_en = GPIO_PULLUP_ENABLE,
                      .pull_down_en = GPIO_PULLDOWN_DISABLE,
                      .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io);
}

// Симетричний дебаунс: стан змінюється лише після N однакових вибірок поспіль,
// тож одинична завада від мотора не "телепортує" X і не блокує рух.
static void debounce(bool raw, int *cnt, bool *state) {
  if (raw != *state) {
    if (++(*cnt) >= STOPPER_DEBOUNCE_SAMPLES) {
      *state = raw;
      *cnt = 0;
    }
  } else {
    *cnt = 0;
  }
}

void stoppers_update(void) {
  debounce(stopper_left_raw(), &s_left_cnt, &s_left);
  debounce(stopper_right_raw(), &s_right_cnt, &s_right);
}

bool stopper_left_pressed(void) { return s_left; }
bool stopper_right_pressed(void) { return s_right; }
bool stopper_left_raw(void) { return gpio_get_level(STOPPER_LEFT_PIN) == 0; }
bool stopper_right_raw(void) { return gpio_get_level(STOPPER_RIGHT_PIN) == 0; }
