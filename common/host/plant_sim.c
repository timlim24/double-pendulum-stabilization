// =============================================================================
// Модель стенду (тільки для ПК): кроковий візок + маятник + енкодер + стопери.
//
// Візок кінематичний: генератор кроків виконує командну швидкість (з мертвою
// зоною MIN_STEP_FREQ і квантуванням частоти), механіка відстає з постійною
// часу τ (пас/ротор). Положення, яке бачить контролер, — лічильник кроків,
// тому при упиранні в стопер "вимір" і реальність розходяться, як на залізі.
//
// Маятник (φ — від низу, знакова конвенція стенду):
//   φ'' = −ω0² sinφ − γ a cosφ − d φ' − c tanh(φ'/ε)
// =============================================================================
#include "cart_pole_config.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define COULOMB_EPS 0.05
#define WALL_OVERTRAVEL 0.003 // м за точкою спрацювання стопера

typedef struct {
  double w2, gamma, d, c, tau;
} pend_params_t;

typedef struct {
  pend_params_t p;
  double x_true, v_act, phi, w;
  double v_step;
  double steps;   // накопичені кроки (дробові)
  double x_off;   // зсув виміряної позиції (калібрування)
  double backlash; // люфт пас/шків (м): вал крутиться, візок стоїть
  double x_motor;  // положення "на валу" (до люфту)
  double v_true_last;
  int32_t enc_off;
  int lost_steps; // ознака: візок упирався
} plant_t;

static double pend_acc(const pend_params_t *p, double phi, double w, double a) {
  return -p->w2 * sin(phi) - p->gamma * a * cos(phi) - p->d * w -
         p->c * tanh(w / COULOMB_EPS);
}

void *plant_create(double w2, double gamma, double d, double c, double tau) {
  plant_t *s = (plant_t *)calloc(1, sizeof(plant_t));
  s->p.w2 = w2;
  s->p.gamma = gamma;
  s->p.d = d;
  s->p.c = c;
  s->p.tau = tau;
  return s;
}

void plant_destroy(void *h) { free(h); }

void plant_set_params(void *h, double w2, double gamma, double d, double c, double tau) {
  plant_t *s = (plant_t *)h;
  s->p.w2 = w2;
  s->p.gamma = gamma;
  s->p.d = d;
  s->p.c = c;
  s->p.tau = tau;
}

void plant_reset(void *h, double x, double phi, double w, int32_t enc_off) {
  plant_t *s = (plant_t *)h;
  s->x_true = x;
  s->v_act = 0.0;
  s->v_step = 0.0;
  s->phi = phi;
  s->w = w;
  s->steps = 0.0;
  s->x_off = 0.0; // до хомінгу контролер бачить X відносно точки старту
  s->x_motor = x;
  s->v_true_last = 0.0;
  s->enc_off = enc_off;
  s->lost_steps = 0;
}

void plant_kick(void *h, double dw) { ((plant_t *)h)->w += dw; }

void plant_set_backlash(void *h, double b) {
  plant_t *s = (plant_t *)h;
  s->backlash = b;
  s->x_motor = s->x_true;
}

void plant_set_measured_x(void *h, double value) {
  plant_t *s = (plant_t *)h;
  s->x_off = value - s->steps / (double)STEPS_PER_METER;
}

// Один такт контролера: v_cmd діє dt секунд.
void plant_step(void *h, double v_cmd, int hard_stop, double dt) {
  plant_t *s = (plant_t *)h;
  const pend_params_t *p = &s->p;

  // Генератор кроків: мертва зона + ціла частота
  double f = fabs(v_cmd) * (double)STEPS_PER_METER;
  s->v_step = (f < (double)MIN_STEP_FREQ) ? 0.0 : copysign(floor(f), v_cmd) / STEPS_PER_METER;
  if (hard_stop) {
    // Ротор зупиняється різко: імпульс на маятник
    s->w += -p->gamma * (0.0 - s->v_act) * cos(s->phi);
    s->v_act = 0.0;
  }
  s->steps += s->v_step * (double)STEPS_PER_METER * dt;

  int n = 8;
  double tau = p->tau;
  if (tau > 1e-5) {
    int need = (int)ceil(dt / (0.5 * tau));
    if (need > n)
      n = need;
    if (n > 64)
      n = 64;
  }
  double hh = dt / n;
  const double xl = -(double)RAIL_CENTER_M - WALL_OVERTRAVEL;
  const double xr = (double)RAIL_CENTER_M + WALL_OVERTRAVEL;

  const bool lash = s->backlash > 0.0;
  for (int i = 0; i < n; i++) {
    if (tau <= 1e-5) {
      // Миттєвий привід: стрибок швидкості = імпульс прискорення
      double dv = s->v_step - s->v_act;
      if (!lash)
        s->w += -p->gamma * dv * cos(s->phi);
      s->v_act = s->v_step;
    }
    // RK4 по (x_вала, v_вала, phi, w). Без люфту візок = вал, маятник
    // відчуває прискорення вала. З люфтом прискорення візка подається
    // імпульсами нижче, тому в RK4 a_маятника = 0.
    double y[4] = {s->x_motor, s->v_act, s->phi, s->w}, k[4][4];
    for (int st = 0; st < 4; st++) {
      double yy[4];
      double fac = (st == 0) ? 0.0 : (st == 3 ? 1.0 : 0.5);
      for (int j = 0; j < 4; j++)
        yy[j] = y[j] + (st == 0 ? 0.0 : fac * hh * k[st - 1][j]);
      double a = (tau > 1e-5) ? (s->v_step - yy[1]) / tau : 0.0;
      k[st][0] = yy[1];
      k[st][1] = a;
      k[st][2] = yy[3];
      k[st][3] = pend_acc(p, yy[2], yy[3], lash ? 0.0 : a);
    }
    for (int j = 0; j < 4; j++)
      y[j] += hh / 6.0 * (k[0][j] + 2 * k[1][j] + 2 * k[2][j] + k[3][j]);
    s->x_motor = y[0];
    s->v_act = y[1];
    s->phi = y[2];
    s->w = y[3];
    if (lash) {
      // Люфт пас/шків: візок рушає, лише коли вал вибрав зазор
      double x_prev = s->x_true;
      double half = 0.5 * s->backlash;
      if (s->x_motor - s->x_true > half)
        s->x_true = s->x_motor - half;
      else if (s->x_motor - s->x_true < -half)
        s->x_true = s->x_motor + half;
      double v_true = (s->x_true - x_prev) / hh;
      s->w += -p->gamma * (v_true - s->v_true_last) * cos(s->phi);
      s->v_true_last = v_true;
    } else {
      s->x_true = s->x_motor;
    }

    // Фізичні стінки за стоперами: візок стоїть, кроки "губляться"
    if ((s->x_true <= xl && s->v_act < 0) || (s->x_true >= xr && s->v_act > 0)) {
      s->x_true = s->x_true <= xl ? xl : xr;
      double v_c = lash ? s->v_true_last : s->v_act;
      s->w += -p->gamma * (0.0 - v_c) * cos(s->phi);
      s->v_act = 0.0;
      s->v_true_last = 0.0;
      s->x_motor = s->x_true;
      s->lost_steps = 1;
    }
  }
}

// Стан: [x_true, v_act, phi, w, x_meas, enc_ticks, stop_l, stop_r]
void plant_get(void *h, double *o8) {
  plant_t *s = (plant_t *)h;
  o8[0] = s->x_true;
  o8[1] = s->v_act;
  o8[2] = s->phi;
  o8[3] = s->w;
  o8[4] = floor(s->steps) / (double)STEPS_PER_METER + s->x_off;
  o8[5] = (double)((int32_t)floor(s->phi / (double)RAD_PER_TICK + 0.5) + s->enc_off);
  o8[6] = s->x_true <= -(double)RAIL_CENTER_M ? 1.0 : 0.0;
  o8[7] = s->x_true >= (double)RAIL_CENTER_M ? 1.0 : 0.0;
}

// -----------------------------------------------------------------------------
// Відкритий контур для SysID: швидкість візка задана відліками v_in (лінійна
// інтерполяція між ними), повертає φ(t). params = [w2, gamma, d, c, tau].
// -----------------------------------------------------------------------------
void plant_simulate(const double *params, double phi0, double w0, int n, double dt,
                    const double *v_in, double *phi_out) {
  pend_params_t p = {params[0], params[1], params[2], params[3], params[4]};
  double tau = p.tau < 2e-4 ? 2e-4 : p.tau;
  int sub = (int)ceil(dt / (0.5 * tau));
  if (sub < 4)
    sub = 4;
  if (sub > 64)
    sub = 64;
  double hh = dt / sub;
  double v = v_in[0], phi = phi0, w = w0;
  phi_out[0] = phi;
  for (int i = 0; i + 1 < n; i++) {
    double v0 = v_in[i], slope = (v_in[i + 1] - v_in[i]) / dt;
    for (int sstep = 0; sstep < sub; sstep++) {
      double t0 = sstep * hh;
      double y[3] = {v, phi, w}, k[4][3];
      for (int st = 0; st < 4; st++) {
        double fac = (st == 0) ? 0.0 : (st == 3 ? 1.0 : 0.5);
        double tt = t0 + fac * hh;
        double yy[3];
        for (int j = 0; j < 3; j++)
          yy[j] = y[j] + (st == 0 ? 0.0 : fac * hh * k[st - 1][j]);
        double vin = v0 + slope * tt;
        double a = (vin - yy[0]) / tau;
        k[st][0] = a;
        k[st][1] = yy[2];
        k[st][2] = pend_acc(&p, yy[1], yy[2], a);
      }
      for (int j = 0; j < 3; j++)
        y[j] += hh / 6.0 * (k[0][j] + 2 * k[1][j] + 2 * k[2][j] + k[3][j]);
      v = y[0];
      phi = y[1];
      w = y[2];
    }
    phi_out[i + 1] = phi;
  }
}
