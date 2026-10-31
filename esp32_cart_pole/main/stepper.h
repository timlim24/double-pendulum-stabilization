#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ініціалізація драйвера крокового двигуна TB6560:
 *        GPIO для напрямку (CW) та дозволу (EN), а також LEDC таймер/канал для
 * генерації кроків (CLK)
 */
void stepper_init(void);

/**
 * @brief Встановлення бажаної лінійної швидкості візка в м/с
 *        Автоматично перевіряє стопери і не дозволяє рух в бік натиснутого
 * кінцевика
 * @param vel_mps Швидкість у м/с (додатна = вправо, від'ємна = вліво)
 */
void stepper_set_velocity(float vel_mps);

/**
 * @brief Повна зупинка генерації імпульсів крокового двигуна
 */
void stepper_stop(void);

/**
 * @brief Управління утриманням валу (Enable)
 * @param enable true = обмотки під струмом (утримання), false = вільне
 * обертання
 */
void stepper_enable(bool enable);

/**
 * @brief Отримання поточної встановленої лінійної швидкості візка
 * @return Швидкість у м/с
 */
float stepper_get_velocity(void);

#ifdef __cplusplus
}
#endif
