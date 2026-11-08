#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Квадратурний енкодер маятника на PCNT (x4, з накопиченням переповнень).
void encoder_init(void);

/// Сирий накопичений лічильник (нуль/низ веде ядро керування).
int32_t encoder_get_count(void);

#ifdef __cplusplus
}
#endif
