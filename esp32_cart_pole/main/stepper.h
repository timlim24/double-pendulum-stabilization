#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Драйвер крокового двигуна (TB6560).
 *  - Кроки генерує апаратний LEDC на піні CLK.
 *  - Ті самі імпульси рахує PCNT (вхід з того ж піна, напрямок — рівень DIR),
 *    тому позиція візка — це кількість РЕАЛЬНО виданих кроків, а не інтеграл
 *    команди. Плавність розгону/гальмування забезпечує ядро керування.
 */
void stepper_init(void);

/// Задати швидкість (м/с). Нижче MIN_STEP_FREQ генератор вимикається.
void stepper_set_velocity(float vel_mps);

/// Миттєва зупинка генератора (тільки при упиранні у стопер).
void stepper_stop(void);

/// true = обмотки під струмом (утримання), false = вільне обертання
void stepper_enable(bool enable);

/// Позиція візка (м) з лічильника виданих кроків + калібрування.
float stepper_get_position_m(void);

/// Перекалібрувати позицію (стопер / хомінг / ручний центр).
void stepper_set_position_m(float x_m);

/// Викликати кожен такт контуру: оновлює резервну (розрахункову) позицію.
void stepper_tick(float dt);

/// Діагностика: кроки з PCNT та розрахункові кроки (мають майже збігатися).
int32_t stepper_get_counted_steps(void);
int32_t stepper_get_estimated_steps(void);
/// false — PCNT не бачить кроків, використовується розрахункова позиція.
bool stepper_counter_ok(void);

#ifdef __cplusplus
}
#endif
