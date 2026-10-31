#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void encoder_init(void);
void encoder_reset(void);
int encoder_get_raw_count(void);
int encoder_get_raw_hardware_count(void);
float encoder_calculate_bottom_angle(int raw_ticks);
float encoder_calculate_theta(int raw_ticks);
float encoder_get_theta(void);

#ifdef __cplusplus
}
#endif
