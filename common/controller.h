#pragma once
// =============================================================================
// ЯДРО КЕРУВАННЯ CART-POLE (портативний C, без залежностей від ESP-IDF)
//
// Один і той самий код виконується:
//   * на ESP32 (esp32_cart_pole/main/main.c викликає ctrl_step() на 500 Гц)
//   * у симуляції на ПК (single_pendulum/ через ctypes, controller_host_api.c)
// Тому симуляція поводиться так само, як стенд: ті ж стани, пороги, фільтри,
// регулятори, обмеження швидкості/прискорення.
//
// Знакова конвенція (перевірена на стенді):
//   x  > 0 — вправо (DIR = 1), лівий стопер у x = -RAIL_CENTER_M
//   θ  — кут від ВЕРХНЬОЇ вертикалі, (-π, π]
//   φ  — кут від НИЖНЬОЇ вертикалі, (-π, π]
//   θ'' = ω0² sinθ + γ a cosθ − ...   (a — прискорення візка)
// =============================================================================

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ST_HOMING_SEEK = 0, // пошук стопера (калібрування X)
  ST_HOMING_CENTER,   // виїзд у центр
  ST_ZEROING,         // гасіння коливань внизу + центрування
  ST_WAITING,         // спокій внизу, очікування команди
  ST_SWINGUP,         // підйом за траєкторією + TVLQR
  ST_BALANCING,       // LQR у верхній точці
  ST_STOPPING,        // плавне гальмування до нуля
  ST_SYSID,           // автономна ідентифікація
  ST_COUNT
} ctrl_state_t;

// Команди (бітова маска в ctrl_input_t.cmd)
enum {
  CMD_SWINGUP = 1u << 0,    // 's'
  CMD_SET_CENTER = 1u << 1, // 'c' — прийняти поточне місце за X = 0
  CMD_HOME = 1u << 2,       // 'h'
  CMD_SYSID = 1u << 3,      // 'i'
  CMD_ABORT = 1u << 4,      // 'x' — зупинка, повернення до гасіння
};

typedef enum {
  EV_NONE = 0,
  EV_BOOT,
  EV_HOMED,             // a = нова X (м), b = корекція (мм)
  EV_HOMING_NO_STOPPER, // стопер не знайдено, X = 0 тут
  EV_CENTERED,
  EV_SETTLED,
  EV_DISTURBED,         // a = кут°, b = ω
  EV_SWINGUP_START,
  EV_SWINGUP_ABORT,     // a = причина (1 = X, 2 = кут), b = t, c = значення
  EV_SWINGUP_END,       // траєкторія скінчилась без захоплення
  EV_CATCH,             // a = θ°, b = ω, c = X
  EV_FALL,              // a = θ°
  EV_SOFT_LIMIT,        // a = X
  EV_STOPPER,           // a = сторона (-1/+1), b = корекція X (мм), c = стан
  EV_ZERO_CORRECTED,    // a = корекція нуля (°)
  EV_SYSID_START,
  EV_SYSID_TRIAL_DONE,  // a = номер тріалу, b = к-сть зразків
  EV_SYSID_NEXT,        // a = номер тріалу
  EV_SYSID_ALL_DONE,
  EV_CMD_REJECTED,      // a = код команди, b = стан
  EV_MANUAL_CENTER,     // a = стара X
  EV_ABORT_CMD,         // команда "стоп", a = стан
  EV_COUNT
} ctrl_event_code_t;

typedef struct {
  int code;
  float a, b, c;
} ctrl_event_t;

typedef struct {
  int32_t enc_ticks; // сирий лічильник енкодера (з накопиченням переповнень)
  float cart_x;      // м, з лічильника кроків (+ калібрування)
  bool stop_left;    // кінцевики після дебаунсу
  bool stop_right;
  uint32_t cmd;           // CMD_* на цьому такті
  bool sysid_buffer_free; // буфер SysID вивантажено
} ctrl_input_t;

#define CTRL_MAX_EVENTS 4

typedef struct {
  float v_cmd;      // м/с — швидкість для генератора кроків (вже з обмеженням прискорення)
  bool hard_stop;   // миттєва зупинка (тільки при фізичному контакті зі стопером)
  bool set_x;       // запит перекалібрувати X візка
  float set_x_value;
  int n_events;
  ctrl_event_t events[CTRL_MAX_EVENTS];
  // SysID: зразок для запису (250 Гц)
  bool sysid_sample;
  int sysid_trial;
  int sysid_index;
  float sysid_x, sysid_v;
  int32_t sysid_ticks; // тіки відносно нуля (0 = низ)
  bool sysid_trial_done; // тріал завершено -> вивантажити буфер
} ctrl_output_t;

typedef struct {
  float k_x, k_th, k_v, k_w, k_i;
} ctrl_gains_t;

typedef struct {
  // ---- Публічна телеметрія (тільки читати) ----
  ctrl_state_t state;
  float t;         // час від старту (с)
  float x;         // позиція візка (м)
  float v;         // командна швидкість (м/с)
  float a;         // останнє командне прискорення (м/с^2)
  float theta;     // від верху, (-π, π]
  float phi;       // від низу, (-π, π]
  float omega;     // оцінка кутової швидкості (рад/с)
  int32_t enc_zero; // тіки, що відповідають низу
  bool homed;
  bool stop_left, stop_right;
  float swing_t;   // час відтворення траєкторії
  int sysid_trial;

  // ---- Внутрішнє ----
  ctrl_gains_t gains;
  float ang_est;   // нерозгорнутий кут від низу (оцінка)
  float omega_est;
  float x_int;     // інтеграл похибки X у балансі
  float state_t;   // час у поточному стані
  float settle_t;
  float homing_start_x;
  bool pending_swingup, pending_sysid;
  float pending_t;
  bool prev_left, prev_right;
  bool catch_armed; // перехоплення дозволене лише після справжнього падіння
  // детектор спокою (автокорекція нуля)
  float rest_t;
  int32_t rest_min, rest_max;
  // SysID
  int sysid_sub;   // 0 = збудження, 1 = заспокоєння
  float sysid_tt;
  int sysid_k;
  bool sysid_buffer_free;
} ctrl_t;

void ctrl_init(ctrl_t *c, int32_t enc_ticks_at_boot);
void ctrl_step(ctrl_t *c, const ctrl_input_t *in, ctrl_output_t *out);

const char *ctrl_state_name(ctrl_state_t s);
int ctrl_event_format(const ctrl_event_t *ev, char *buf, int n);

// Профіль збудження SysID (позиція та швидкість візка), тривалість тріалу
float ctrl_sysid_duration(int trial);
void ctrl_sysid_profile(int trial, float t, float *x, float *v);

#ifdef __cplusplus
}
#endif
