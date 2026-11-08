// =============================================================================
// ESP32 Cart-Pole: обгортка над ядром керування common/controller.c
//
//  * control_task (ядро 1, макс. пріоритет, 500 Гц): датчики -> ctrl_step() ->
//    генератор кроків. НІЧОГО не друкує (друк по UART блокує на мілісекунди).
//  * io_task (ядро 0): команди з UART, події, телеметрія 10 Гц, вивантаження
//    записів SysID.
//
// Команди з консолі (cli_control.py або idf.py monitor):
//   s — підйом, x / пробіл — стоп, h — хомінг, c — поточне місце = центр,
//   i — SysID
// =============================================================================
#include "cart_pole_config.h"
#include "controller.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stepper.h"
#include "stoppers.h"
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static const char *TAG = "CART_POLE";

// ---- Обмін між задачами ----
static volatile uint32_t s_cmd_bits = 0; // встановлює io_task, забирає control_task
static QueueHandle_t s_event_q;
static volatile uint32_t s_events_dropped = 0;

typedef struct {
  ctrl_state_t state;
  float x, v, theta, phi, omega;
  int32_t enc_raw, enc_zero;
  bool homed, stop_l, stop_r, raw_l, raw_r;
  int32_t steps_cnt, steps_est;
  bool counter_ok;
  uint32_t loop_us_max, period_us_max;
  int sysid_trial;
} telemetry_t;

static telemetry_t s_tel;
static portMUX_TYPE s_tel_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- Буфер SysID ----
typedef struct {
  float x, v;
  int32_t ticks;
} sysid_rec_t;
static sysid_rec_t s_sysid_buf[SYSID_MAX_SAMPLES];
static volatile int s_sysid_n = 0;
static volatile int s_sysid_trial = 0;
static volatile bool s_sysid_dump_pending = false;
static volatile bool s_sysid_buffer_free = true;

// =============================================================================
// КОНТУР КЕРУВАННЯ 500 Гц
// =============================================================================
static void control_task(void *arg) {
  static ctrl_t ctrl;
  ctrl_input_t in;
  ctrl_output_t out;

  ctrl_init(&ctrl, encoder_get_count());

  const TickType_t period = pdMS_TO_TICKS(1000 / CTRL_RATE_HZ);
  TickType_t last_wake = xTaskGetTickCount();
  int64_t t_prev = esp_timer_get_time();
  uint32_t loop_max = 0, period_max = 0;

  while (1) {
    vTaskDelayUntil(&last_wake, period > 0 ? period : 1);
    int64_t t0 = esp_timer_get_time();
    uint32_t per = (uint32_t)(t0 - t_prev);
    t_prev = t0;
    if (per > period_max)
      period_max = per;

    // 1. Датчики
    stoppers_update();
    in.enc_ticks = encoder_get_count();
    in.cart_x = stepper_get_position_m();
    in.stop_left = stopper_left_pressed();
    in.stop_right = stopper_right_pressed();
    in.cmd = __atomic_exchange_n(&s_cmd_bits, 0, __ATOMIC_ACQ_REL);
    in.sysid_buffer_free = s_sysid_buffer_free;

    // 2. Ядро керування (той самий код, що в симуляції)
    ctrl_step(&ctrl, &in, &out);

    // 3. Виконання
    if (out.set_x)
      stepper_set_position_m(out.set_x_value);
    if (out.hard_stop)
      stepper_stop();
    stepper_set_velocity(out.v_cmd);
    stepper_tick(CTRL_DT);

    // 4. Події -> черга (друкує io_task)
    for (int i = 0; i < out.n_events; i++) {
      if (xQueueSend(s_event_q, &out.events[i], 0) != pdTRUE)
        s_events_dropped++;
    }

    // 5. SysID: запис у RAM, вивантаження після тріалу
    if (out.sysid_sample) {
      if (out.sysid_index == 0)
        s_sysid_n = 0;
      if (s_sysid_n < SYSID_MAX_SAMPLES) {
        s_sysid_buf[s_sysid_n].x = out.sysid_x;
        s_sysid_buf[s_sysid_n].v = out.sysid_v;
        s_sysid_buf[s_sysid_n].ticks = out.sysid_ticks;
        s_sysid_n++;
      }
      s_sysid_trial = out.sysid_trial;
    }
    if (out.sysid_trial_done) {
      s_sysid_buffer_free = false;
      s_sysid_dump_pending = true;
    }

    // 6. Знімок телеметрії
    uint32_t loop_us = (uint32_t)(esp_timer_get_time() - t0);
    if (loop_us > loop_max)
      loop_max = loop_us;
    taskENTER_CRITICAL(&s_tel_mux);
    s_tel.state = ctrl.state;
    s_tel.x = ctrl.x;
    s_tel.v = ctrl.v;
    s_tel.theta = ctrl.theta;
    s_tel.phi = ctrl.phi;
    s_tel.omega = ctrl.omega;
    s_tel.enc_raw = in.enc_ticks;
    s_tel.enc_zero = ctrl.enc_zero;
    s_tel.homed = ctrl.homed;
    s_tel.stop_l = in.stop_left;
    s_tel.stop_r = in.stop_right;
    s_tel.sysid_trial = ctrl.sysid_trial;
    s_tel.loop_us_max = loop_max;
    s_tel.period_us_max = period_max;
    taskEXIT_CRITICAL(&s_tel_mux);
  }
}

// =============================================================================
// ВВІД/ВИВІД: команди, події, телеметрія, SysID
// =============================================================================
static void handle_input(void) {
  char c;
  while (read(STDIN_FILENO, &c, 1) > 0) {
    uint32_t bit = 0;
    switch (c) {
    case 's':
    case 'S':
      bit = CMD_SWINGUP;
      break;
    case 'x':
    case 'X':
    case ' ':
      bit = CMD_ABORT;
      break;
    case 'h':
    case 'H':
      bit = CMD_HOME;
      break;
    case 'c':
    case 'C':
      bit = CMD_SET_CENTER;
      break;
    case 'i':
    case 'I':
      bit = CMD_SYSID;
      break;
    default:
      break;
    }
    if (bit)
      __atomic_fetch_or(&s_cmd_bits, bit, __ATOMIC_ACQ_REL);
  }
}

static void print_events(void) {
  ctrl_event_t ev;
  char buf[160];
  while (xQueueReceive(s_event_q, &ev, 0) == pdTRUE) {
    ctrl_event_format(&ev, buf, sizeof(buf));
    printf("EVT:%d %s\r\n", ev.code, buf);
  }
  if (s_events_dropped) {
    printf("EVT:0 (втрачено подій: %lu)\r\n", (unsigned long)s_events_dropped);
    s_events_dropped = 0;
  }
}

static void dump_sysid(void) {
  int n = s_sysid_n;
  float dt = CTRL_DT * SYSID_LOG_DECIM;
  printf("SYSID_BEGIN,%d,%d,%.4f\r\n", s_sysid_trial, n, dt);
  for (int i = 0; i < n; i++) {
    printf("D,%d,%.5f,%.4f,%ld\r\n", i, s_sysid_buf[i].x, s_sysid_buf[i].v,
           (long)s_sysid_buf[i].ticks);
    if ((i & 15) == 15)
      vTaskDelay(1); // не голодувати IDLE (watchdog)
  }
  printf("SYSID_END,%d\r\n", s_sysid_trial);
  s_sysid_dump_pending = false;
  s_sysid_buffer_free = true;
}

static void io_task(void *arg) {
  int tick = 0;
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(20));
    handle_input();
    print_events();
    if (s_sysid_dump_pending) {
      dump_sysid();
      continue;
    }
    if (++tick < 5) // телеметрія 10 Гц
      continue;
    tick = 0;

    telemetry_t t;
    taskENTER_CRITICAL(&s_tel_mux);
    t = s_tel;
    taskEXIT_CRITICAL(&s_tel_mux);

    float deg_top = t.theta * 180.0f / (float)M_PI;
    float deg_bot = t.phi * 180.0f / (float)M_PI;
    printf("[%s] X:%+6.3fm | V:%+5.2fm/s | L:%d R:%d | Кут:%+6.1f° (вниз:%+6.1f°) | "
           "w:%+5.2f | Тіки:%ld Нуль:%ld | Кроки:%ld/%ld%s | %s | цикл:%luмкс\r\n",
           ctrl_state_name(t.state), t.x, t.v, t.stop_l ? 0 : 1, t.stop_r ? 0 : 1, deg_top,
           deg_bot, t.omega, (long)t.enc_raw, (long)t.enc_zero,
           (long)stepper_get_counted_steps(), (long)stepper_get_estimated_steps(),
           stepper_counter_ok() ? "" : "(!PCNT)", t.homed ? "home OK" : "НЕ КАЛІБР.",
           (unsigned long)t.loop_us_max);
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Cart-Pole: рейка %.1f см, хід %.1f см, контур %d Гц", RAIL_BETWEEN_STOPPERS_M * 100.0f,
           RAIL_DISTANCE_M * 100.0f, CTRL_RATE_HZ);
  if (configTICK_RATE_HZ < CTRL_RATE_HZ)
    ESP_LOGE(TAG, "CONFIG_FREERTOS_HZ=%d < %d — контур не встигне! Встановіть 1000.",
             configTICK_RATE_HZ, CTRL_RATE_HZ);

  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

  s_event_q = xQueueCreate(32, sizeof(ctrl_event_t));
  stoppers_init();
  stepper_init();
  encoder_init();
  vTaskDelay(pdMS_TO_TICKS(300)); // маятник має висіти спокійно: старт = низ

  xTaskCreatePinnedToCore(control_task, "control", 6144, NULL, configMAX_PRIORITIES - 1, NULL, 1);
  xTaskCreatePinnedToCore(io_task, "io", 6144, NULL, 3, NULL, 0);
}
