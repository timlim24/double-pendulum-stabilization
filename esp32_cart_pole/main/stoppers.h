#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ініціалізація GPIO кінцевиків (стоперів) з внутрішньою підтяжкою
 * PULLUP
 */
void stoppers_init(void);

/**
 * @brief Оновлення цифрового фільтра та дебаунсу стоперів (викликається в циклі
 * 100 Гц)
 */
void stoppers_update(void);

/**
 * @brief Перевірка відфільтрованого стану лівого стопера
 * @return true якщо лівий стопер надійно натиснуто (Active LOW) без перешкод
 * ЕМЗ
 */
bool stopper_left_pressed(void);

/**
 * @brief Перевірка відфільтрованого стану правого стопера
 * @return true якщо правий стопер надійно натиснуто (Active LOW) без перешкод
 * ЕМЗ
 */
bool stopper_right_pressed(void);

/**
 * @brief Миттєве (без фільтра) зчитування піна лівого стопера
 */
bool stopper_left_raw(void);

/**
 * @brief Миттєве (без фільтра) зчитування піна правого стопера
 */
bool stopper_right_raw(void);

#ifdef __cplusplus
}
#endif
