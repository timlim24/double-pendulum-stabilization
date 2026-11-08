// =============================================================================
// Плоский C API ядра керування для Python (ctypes). Тільки для ПК.
// Збирається single_pendulum/controller_lib.py разом з controller.c та plant_sim.c
// =============================================================================
#include "cart_pole_config.h"
#include "controller.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
  ctrl_t c;
  ctrl_output_t out;
} api_t;

void *api_create(int32_t enc_ticks) {
  api_t *a = (api_t *)calloc(1, sizeof(api_t));
  ctrl_init(&a->c, enc_ticks);
  return a;
}

void api_destroy(void *h) { free(h); }

void api_step(void *h, int32_t ticks, float cart_x, int stop_l, int stop_r, uint32_t cmd,
              int buffer_free) {
  api_t *a = (api_t *)h;
  ctrl_input_t in = {.enc_ticks = ticks,
                     .cart_x = cart_x,
                     .stop_left = stop_l != 0,
                     .stop_right = stop_r != 0,
                     .cmd = cmd,
                     .sysid_buffer_free = buffer_free != 0};
  ctrl_step(&a->c, &in, &a->out);
}

// Вихід останнього кроку: [v_cmd, hard_stop, set_x, set_x_value]
void api_output(void *h, float *o4) {
  api_t *a = (api_t *)h;
  o4[0] = a->out.v_cmd;
  o4[1] = a->out.hard_stop ? 1.0f : 0.0f;
  o4[2] = a->out.set_x ? 1.0f : 0.0f;
  o4[3] = a->out.set_x_value;
}

// Телеметрія: [state, t, x, v, a, theta, phi, omega, homed, swing_t, sysid_trial, enc_zero]
void api_telemetry(void *h, float *o12) {
  const ctrl_t *c = &((api_t *)h)->c;
  o12[0] = (float)c->state;
  o12[1] = c->t;
  o12[2] = c->x;
  o12[3] = c->v;
  o12[4] = c->a;
  o12[5] = c->theta;
  o12[6] = c->phi;
  o12[7] = c->omega;
  o12[8] = c->homed ? 1.0f : 0.0f;
  o12[9] = c->swing_t;
  o12[10] = (float)c->sysid_trial;
  o12[11] = (float)c->enc_zero;
}

int api_n_events(void *h) { return ((api_t *)h)->out.n_events; }

int api_event(void *h, int i, int *code, char *buf, int n) {
  api_t *a = (api_t *)h;
  if (i < 0 || i >= a->out.n_events)
    return -1;
  *code = a->out.events[i].code;
  return ctrl_event_format(&a->out.events[i], buf, n);
}

// SysID: [sample?, trial, index, x, v, ticks, trial_done]
void api_sysid(void *h, float *o7) {
  const ctrl_output_t *o = &((api_t *)h)->out;
  o7[0] = o->sysid_sample ? 1.0f : 0.0f;
  o7[1] = (float)o->sysid_trial;
  o7[2] = (float)o->sysid_index;
  o7[3] = o->sysid_x;
  o7[4] = o->sysid_v;
  o7[5] = (float)o->sysid_ticks;
  o7[6] = o->sysid_trial_done ? 1.0f : 0.0f;
}

void api_get_gains(void *h, float *g5) {
  const ctrl_gains_t *g = &((api_t *)h)->c.gains;
  g5[0] = g->k_x;
  g5[1] = g->k_th;
  g5[2] = g->k_v;
  g5[3] = g->k_w;
  g5[4] = g->k_i;
}

void api_set_gains(void *h, const float *g5) {
  ctrl_gains_t *g = &((api_t *)h)->c.gains;
  g->k_x = g5[0];
  g->k_th = g5[1];
  g->k_v = g5[2];
  g->k_w = g5[3];
  g->k_i = g5[4];
}

const char *api_state_name(int s) { return ctrl_state_name((ctrl_state_t)s); }

void api_sysid_profile(int trial, float t, float *x, float *v) {
  ctrl_sysid_profile(trial, t, x, v);
}

float api_sysid_duration(int trial) { return ctrl_sysid_duration(trial); }

// Корисні константи для Python
float api_const(const char *name) {
#define C(n)                                                                                       \
  if (strcmp(name, #n) == 0)                                                                       \
    return (float)(n);
  C(CTRL_DT)
  C(CTRL_RATE_HZ)
  C(RAIL_CENTER_M)
  C(STEPS_PER_METER)
  C(RAD_PER_TICK)
  C(MIN_STEP_FREQ)
  C(SYSID_LOG_DECIM)
  C(SYSID_NUM_TRIALS)
#undef C
  return -1.0f;
}
