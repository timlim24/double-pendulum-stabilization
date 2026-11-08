#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Ініціалізація кінцевиків (Active LOW, внутрішня підтяжка)
void stoppers_init(void);

/// Дебаунс; викликати кожен такт контуру керування
void stoppers_update(void);

/// Стан після дебаунсу (STOPPER_DEBOUNCE_SAMPLES поспіль). Для логіки — тільки ці.
bool stopper_left_pressed(void);
bool stopper_right_pressed(void);

/// Сирий рівень (тільки для діагностики в телеметрії)
bool stopper_left_raw(void);
bool stopper_right_raw(void);

#ifdef __cplusplus
}
#endif
