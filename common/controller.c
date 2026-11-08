// =============================================================================
// ЯДРО КЕРУВАННЯ CART-POLE — див. controller.h
// =============================================================================
#include "controller.h"
#include "cart_pole_config.h"
#include "pendulum_params.h"
#include "swingup_trajectory.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI_F ((float)M_PI)
#define TWO_PI_F (2.0f * (float)M_PI)

// -----------------------------------------------------------------------------
// Допоміжні функції
// -----------------------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline float wrap_pi(float a) {
  a = fmodf(a + PI_F, TWO_PI_F);
  if (a < 0.0f)
    a += TWO_PI_F;
  return a - PI_F;
}

static inline float signf_(float v) { return v > 0.0f ? 1.0f : (v < 0.0f ? -1.0f : 0.0f); }

static void emit(ctrl_output_t *out, int code, float a, float b, float c) {
  if (out->n_events < CTRL_MAX_EVENTS) {
    ctrl_event_t *e = &out->events[out->n_events++];
    e->code = code;
    e->a = a;
    e->b = b;
    e->c = c;
  }
}

static void set_state(ctrl_t *c, ctrl_state_t s) {
  c->state = s;
  c->state_t = 0.0f;
  c->settle_t = 0.0f;
  switch (s) {
  case ST_HOMING_SEEK:
    c->homing_start_x = c->x;
    break;
  case ST_BALANCING:
    c->x_int = 0.0f;
    break;
  case ST_SWINGUP:
    c->swing_t = 0.0f;
    break;
  case ST_STOPPING:
    // Маятник може ще стояти вертикально (скасування, межа рейки) — не
    // перехоплювати знову, доки він справді не впаде.
    c->catch_armed = false;
    break;
  default:
    break;
  }
}

// Обмеження швидкості гальмівним шляхом до меж [lo, hi]: візок завжди встигає
// зупинитися з прискоренням a_brake, тож миттєві зупинки не потрібні.
static float shape_velocity(float v, float x, float lo, float hi, float a_brake) {
  float d_hi = hi - x, d_lo = x - lo;
  float v_hi = d_hi > 0.0f ? sqrtf(2.0f * a_brake * d_hi) : 0.0f;
  float v_lo = d_lo > 0.0f ? sqrtf(2.0f * a_brake * d_lo) : 0.0f;
  if (v > v_hi)
    v = v_hi;
  if (v < -v_lo)
    v = -v_lo;
  return v;
}

// Плавний переїзд у точку (трапецієподібний профіль швидкості)
static float move_to(const ctrl_t *c, float x_goal, float v_max, float a_max) {
  float d = x_goal - c->x;
  float ad = fabsf(d);
  float v = fminf(v_max, fminf(sqrtf(2.0f * 0.8f * a_max * ad), 3.0f * ad));
  return signf_(d) * v;
}

// Енергетичне гасіння коливань внизу + центрування візка.
// dE/dt = -γ a cosφ ω  =>  a = K ω cosφ завжди забирає енергію (і при обертанні).
static float down_accel(const ctrl_t *c) {
  float a_d = clampf(DOWN_K_DAMP * c->omega * cosf(c->phi), -DOWN_DAMP_ACCEL, DOWN_DAMP_ACCEL);
  float a_c = -DOWN_K_X * c->x - DOWN_K_V * c->v;
  return clampf(a_d + a_c, -DOWN_MAX_ACCEL, DOWN_MAX_ACCEL);
}

static bool is_settled_down(const ctrl_t *c) {
  return fabsf(c->phi) < SETTLE_ANGLE_RAD && fabsf(c->omega) < SETTLE_OMEGA &&
         fabsf(c->x) < SETTLE_X_M && fabsf(c->v) < 0.005f;
}

// -----------------------------------------------------------------------------
// Профілі збудження SysID (відкритий контур, гладкі, починаються і закінчуються
// в X = 0 з нульовою швидкістю)
// -----------------------------------------------------------------------------
#define SYSID1_AMP 0.040f
#define SYSID1_PERIODS 3.0f
#define SYSID2_AMP 0.015f
#define SYSID2_F0 0.40f
#define SYSID2_F1 1.80f
#define SYSID2_TC 9.0f
#define SYSID2_RAMP 1.0f

float ctrl_sysid_duration(int trial) {
  switch (trial) {
  case 1:
    return 11.0f;
  case 2:
    return 11.0f;
  case 3:
    return 9.0f;
  default:
    return 0.0f;
  }
}

static void smooth_move(float t, float t0, float dur, float x0, float x1, float *x, float *v) {
  if (t <= t0) {
    *x = x0;
    *v = 0.0f;
    return;
  }
  if (t >= t0 + dur) {
    *x = x1;
    *v = 0.0f;
    return;
  }
  float u = (t - t0) / dur;
  float s = u * u * u * (10.0f - 15.0f * u + 6.0f * u * u);
  float ds = 30.0f * u * u * (1.0f - u) * (1.0f - u) / dur;
  *x = x0 + (x1 - x0) * s;
  *v = (x1 - x0) * ds;
}

void ctrl_sysid_profile(int trial, float t, float *x, float *v) {
  *x = 0.0f;
  *v = 0.0f;
  if (t < 0.0f)
    return;
  if (trial == 1) {
    // Резонансне розгойдування (3 періоди) + вільне згасання з нерухомим візком
    float tex = SYSID1_PERIODS * TWO_PI_F / PEND_OMEGA0;
    if (t >= tex)
      return;
    float w = sinf(PI_F * t / tex), wv = w * w;
    float dw = (PI_F / tex) * sinf(TWO_PI_F * t / tex);
    *x = SYSID1_AMP * sinf(PEND_OMEGA0 * t) * wv;
    *v = SYSID1_AMP * (PEND_OMEGA0 * cosf(PEND_OMEGA0 * t) * wv + sinf(PEND_OMEGA0 * t) * dw);
  } else if (trial == 2) {
    // Лінійний chirp 0.4 → 1.8 Гц з плавним вікном
    if (t >= SYSID2_TC)
      return;
    float k = (SYSID2_F1 - SYSID2_F0) / SYSID2_TC;
    float ph = TWO_PI_F * (SYSID2_F0 * t + 0.5f * k * t * t);
    float f = SYSID2_F0 + k * t;
    float w = 1.0f, dw = 0.0f;
    if (t < SYSID2_RAMP) {
      float s = sinf(0.5f * PI_F * t / SYSID2_RAMP);
      w = s * s;
      dw = (0.5f * PI_F / SYSID2_RAMP) * sinf(PI_F * t / SYSID2_RAMP);
    } else if (t > SYSID2_TC - SYSID2_RAMP) {
      float tr = SYSID2_TC - t;
      float s = sinf(0.5f * PI_F * tr / SYSID2_RAMP);
      w = s * s;
      dw = -(0.5f * PI_F / SYSID2_RAMP) * sinf(PI_F * tr / SYSID2_RAMP);
    }
    *x = SYSID2_AMP * sinf(ph) * w;
    *v = SYSID2_AMP * (TWO_PI_F * f * cosf(ph) * w + sinf(ph) * dw);
  } else if (trial == 3) {
    // Ступінчасті переїзди ±5 см (валідаційний тріал)
    if (t < 2.0f)
      smooth_move(t, 0.3f, 0.3f, 0.0f, 0.05f, x, v);
    else if (t < 4.4f)
      smooth_move(t, 2.3f, 0.4f, 0.05f, -0.05f, x, v);
    else
      smooth_move(t, 4.5f, 0.3f, -0.05f, 0.0f, x, v);
  }
}

// -----------------------------------------------------------------------------
// Ініціалізація
// -----------------------------------------------------------------------------
void ctrl_init(ctrl_t *c, int32_t enc_ticks_at_boot) {
  memset(c, 0, sizeof(*c));
  c->enc_zero = enc_ticks_at_boot; // маятник при ввімкненні має висіти; далі — автокорекція
  c->gains.k_x = LQR_K_X;
  c->gains.k_th = LQR_K_TH;
  c->gains.k_v = LQR_K_V;
  c->gains.k_w = LQR_K_W;
  c->gains.k_i = LQR_K_I;
  c->sysid_buffer_free = true;
  c->catch_armed = true;
  c->phi = 0.0f;
  c->theta = PI_F;
  set_state(c, ST_HOMING_SEEK);
}

// -----------------------------------------------------------------------------
// Головний крок (викликається з частотою CTRL_RATE_HZ)
// -----------------------------------------------------------------------------
void ctrl_step(ctrl_t *c, const ctrl_input_t *in, ctrl_output_t *out) {
  const float dt = CTRL_DT;
  memset(out, 0, sizeof(*out));

  if (c->t == 0.0f)
    emit(out, EV_BOOT, 0, 0, 0);
  c->t += dt;
  c->state_t += dt;

  // ---- 1. Вимірювання ----
  c->x = in->cart_x;
  c->stop_left = in->stop_left;
  c->stop_right = in->stop_right;
  c->sysid_buffer_free = in->sysid_buffer_free;

  // Оцінювач кута/кутової швидкості з моделлю (на НЕРОЗГОРНУТОМУ куті, без
  // стрибків ±π). Прогноз — за рівнянням маятника з відомим командним
  // прискоренням візка, енкодер лише коригує похибку моделі. Без моделі
  // альфа-бета фільтр запізнюється на ~25 мс по ω, і цього досить, щоб зірвати
  // підйом (кутове прискорення до 50 рад/с²).
  float meas = (float)(in->enc_ticks - c->enc_zero) * RAD_PER_TICK;
  float acc = -PEND_OMEGA0_SQ * sinf(c->ang_est) - PEND_GAMMA * c->a * cosf(c->ang_est) -
              PEND_D_VISC * c->omega_est;
  float pred = c->ang_est + c->omega_est * dt + 0.5f * acc * dt * dt;
  float pred_w = c->omega_est + acc * dt;
  float r = meas - pred;
  c->ang_est = pred + EST_ALPHA * r;
  c->omega_est = pred_w + (EST_BETA / dt) * r;
  c->omega = c->omega_est;
  c->phi = wrap_pi(c->ang_est);
  c->theta = wrap_pi(c->ang_est + PI_F);

  // ---- 2. Кінцевики: точне калібрування X при спрацюванні ----
  bool left_edge = in->stop_left && !c->prev_left;
  bool right_edge = in->stop_right && !c->prev_right;
  c->prev_left = in->stop_left;
  c->prev_right = in->stop_right;
  if (left_edge || right_edge) {
    float side = left_edge ? -1.0f : 1.0f;
    float x_true = side * RAIL_CENTER_M;
    float corr_mm = (c->x - x_true) * 1000.0f;
    out->set_x = true;
    out->set_x_value = x_true;
    c->x = x_true;
    bool was_homed = c->homed;
    c->homed = true;
    if (signf_(c->v) == side) { // фізично впираємося — зупинка без варіантів
      out->hard_stop = true;
      c->v = 0.0f;
    }
    if (c->state == ST_HOMING_SEEK) {
      emit(out, EV_HOMED, x_true, was_homed ? corr_mm : 0.0f, side);
      set_state(c, ST_HOMING_CENTER);
    } else {
      emit(out, EV_STOPPER, side, was_homed ? corr_mm : 0.0f, (float)c->state);
      if (c->state != ST_HOMING_CENTER && c->state != ST_STOPPING)
        set_state(c, ST_STOPPING);
    }
  }

  // ---- 3. Команди ----
  uint32_t cmd = in->cmd;
  if (cmd & CMD_HOME) {
    c->pending_swingup = c->pending_sysid = false;
    set_state(c, ST_HOMING_SEEK);
  }
  if (cmd & CMD_ABORT) {
    c->pending_swingup = c->pending_sysid = false;
    if (c->state != ST_HOMING_SEEK && c->state != ST_HOMING_CENTER) {
      emit(out, EV_ABORT_CMD, (float)c->state, 0, 0);
      set_state(c, ST_STOPPING);
    }
  }
  if (cmd & CMD_SET_CENTER) {
    if ((c->state == ST_ZEROING || c->state == ST_WAITING) && fabsf(c->v) < 0.01f) {
      emit(out, EV_MANUAL_CENTER, c->x, 0, 0);
      out->set_x = true;
      out->set_x_value = 0.0f;
      c->x = 0.0f;
      c->homed = true;
      set_state(c, ST_ZEROING);
    } else {
      emit(out, EV_CMD_REJECTED, (float)CMD_SET_CENTER, (float)c->state, 0);
    }
  }
  if (cmd & (CMD_SWINGUP | CMD_SYSID)) {
    uint32_t which = (cmd & CMD_SWINGUP) ? CMD_SWINGUP : CMD_SYSID;
    if (c->homed && (c->state == ST_WAITING || c->state == ST_ZEROING)) {
      c->pending_swingup = (which == CMD_SWINGUP);
      c->pending_sysid = (which == CMD_SYSID);
      c->pending_t = 0.0f;
    } else {
      emit(out, EV_CMD_REJECTED, (float)which, (float)c->state, 0);
    }
  }
  if (c->pending_swingup || c->pending_sysid) {
    c->pending_t += dt;
    if (c->pending_t > 15.0f) // не заспокоївся за 15 с — скасувати запит
      c->pending_swingup = c->pending_sysid = false;
  }

  // ---- 4. Глобальне перехоплення у верхній точці ----
  if (fabsf(c->theta) > 1.0f)
    c->catch_armed = true;
  bool catch_ok = c->catch_armed && c->homed && fabsf(c->theta) < GLOBAL_CATCH_ANGLE &&
                  fabsf(c->omega) < GLOBAL_CATCH_OMEGA &&
                  fabsf(c->x) < X_DOWN_LIMIT_M; // ловимо лише з запасом рейки
  bool catch_state = c->state == ST_ZEROING || c->state == ST_WAITING ||
                     c->state == ST_STOPPING ||
                     (c->state == ST_SWINGUP && c->swing_t > SWINGUP_CATCH_FRACTION * SWING_T);
  if (catch_ok && catch_state) {
    emit(out, EV_CATCH, c->theta * 180.0f / PI_F, c->omega, c->x);
    set_state(c, ST_BALANCING);
  }

  // ---- 5. Автомат станів: бажана швидкість та межа прискорення ----
  float v_target = 0.0f;
  float a_lim = STOP_DECEL;
  float lo = -X_SOFT_LIMIT_M, hi = X_SOFT_LIMIT_M;
  bool shape = c->homed;
  float a_cmd = 0.0f;

  switch (c->state) {
  case ST_HOMING_SEEK:
    shape = false;
    a_lim = HOMING_ACCEL;
    v_target = -HOMING_SPEED_MPS;
    if (c->x - c->homing_start_x < -HOMING_MAX_TRAVEL_M) {
      emit(out, EV_HOMING_NO_STOPPER, 0, 0, 0);
      out->set_x = true;
      out->set_x_value = 0.0f;
      c->x = 0.0f;
      c->homed = true;
      set_state(c, ST_STOPPING);
    }
    break;

  case ST_HOMING_CENTER:
    shape = false;
    a_lim = RECENTER_ACCEL;
    v_target = move_to(c, 0.0f, RECENTER_SPEED_MPS, RECENTER_ACCEL);
    if (fabsf(c->x) < 0.002f && fabsf(c->v) < 0.004f) {
      emit(out, EV_CENTERED, c->x, 0, 0);
      set_state(c, ST_ZEROING);
    }
    break;

  case ST_ZEROING:
    a_cmd = down_accel(c);
    a_lim = DOWN_MAX_ACCEL;
    v_target = clampf(c->v + a_cmd * dt, -DOWN_MAX_VELOCITY, DOWN_MAX_VELOCITY);
    lo = -X_DOWN_LIMIT_M;
    hi = X_DOWN_LIMIT_M;
    c->settle_t = is_settled_down(c) ? c->settle_t + dt : 0.0f;
    if (c->settle_t >= SETTLE_TIME_S) {
      emit(out, EV_SETTLED, 0, 0, 0);
      set_state(c, ST_WAITING);
    }
    break;

  case ST_WAITING:
    a_lim = 0.5f;
    v_target = move_to(c, 0.0f, 0.05f, 0.5f);
    lo = -X_DOWN_LIMIT_M;
    hi = X_DOWN_LIMIT_M;
    if (fabsf(c->phi) > DISTURB_ANGLE_RAD || fabsf(c->omega) > DISTURB_OMEGA) {
      emit(out, EV_DISTURBED, c->phi * 180.0f / PI_F, c->omega, 0);
      set_state(c, ST_ZEROING);
    } else if ((c->pending_swingup || c->pending_sysid) && fabsf(c->x) < 0.005f &&
               fabsf(c->v) < 0.003f) {
      if (c->pending_swingup) {
        emit(out, EV_SWINGUP_START, SWING_T, 0, 0);
        set_state(c, ST_SWINGUP);
      } else {
        emit(out, EV_SYSID_START, 0, 0, 0);
        c->sysid_trial = 1;
        c->sysid_sub = 0;
        c->sysid_tt = 0.0f;
        c->sysid_k = 0;
        set_state(c, ST_SYSID);
      }
      c->pending_swingup = c->pending_sysid = false;
      v_target = 0.0f;
    }
    break;

  case ST_SWINGUP: {
    c->swing_t += dt;
    float s = c->swing_t / SWING_DT;
    int i = (int)s;
    float f = s - (float)i;
    if (i >= SWING_N - 1) {
      i = SWING_N - 2;
      f = 1.0f;
    }
    const swing_point_t *p0 = &s_swing_traj[i], *p1 = &s_swing_traj[i + 1];
#define LERP(fld) (p0->fld + f * (p1->fld - p0->fld))
    float e_x = c->x - LERP(x);
    float e_th = wrap_pi(c->theta - LERP(th));
    float e_v = c->v - LERP(v);
    float e_w = c->omega - LERP(w);
    a_cmd = LERP(a) - (LERP(kx) * e_x + LERP(kth) * e_th + LERP(kv) * e_v + LERP(kw) * e_w);
#undef LERP
    a_cmd = clampf(a_cmd, -SWINGUP_MAX_ACCEL, SWINGUP_MAX_ACCEL);
    a_lim = SWINGUP_MAX_ACCEL;
    v_target = clampf(c->v + a_cmd * dt, -MAX_CART_VELOCITY, MAX_CART_VELOCITY);
    lo = -X_SWINGUP_ABORT_M;
    hi = X_SWINGUP_ABORT_M;

    if (fabsf(c->x) > X_SWINGUP_ABORT_M - 0.005f) {
      emit(out, EV_SWINGUP_ABORT, 1.0f, c->swing_t, c->x);
      set_state(c, ST_STOPPING);
    } else if (fabsf(e_th) > SWINGUP_MAX_TH_ERR) {
      emit(out, EV_SWINGUP_ABORT, 2.0f, c->swing_t, e_th * 180.0f / PI_F);
      set_state(c, ST_STOPPING);
    } else if (c->swing_t >= SWING_T) {
      emit(out, EV_SWINGUP_END, c->theta * 180.0f / PI_F, c->omega, 0);
      set_state(c, ST_STOPPING);
    }
    break;
  }

  case ST_BALANCING: {
    c->x_int = clampf(c->x_int + c->x * dt, -BALANCE_I_CLAMP, BALANCE_I_CLAMP);
    const ctrl_gains_t *g = &c->gains;
    a_cmd = -(g->k_x * c->x + g->k_th * c->theta + g->k_v * c->v + g->k_w * c->omega +
              g->k_i * c->x_int);
    a_cmd = clampf(a_cmd, -MAX_CART_ACCEL, MAX_CART_ACCEL);
    a_lim = MAX_CART_ACCEL;
    v_target = clampf(c->v + a_cmd * dt, -BALANCE_MAX_VELOCITY, BALANCE_MAX_VELOCITY);
    if (fabsf(c->theta) > BALANCE_FALL_ANGLE) {
      emit(out, EV_FALL, c->theta * 180.0f / PI_F, c->x, 0);
      set_state(c, ST_STOPPING);
    } else if (fabsf(c->x) > X_SOFT_LIMIT_M - 0.005f) {
      emit(out, EV_SOFT_LIMIT, c->x, 0, 0);
      set_state(c, ST_STOPPING);
    }
    break;
  }

  case ST_STOPPING:
    a_lim = STOP_DECEL;
    v_target = 0.0f;
    lo = -RAIL_CENTER_M;
    hi = RAIL_CENTER_M;
    if (fabsf(c->v) < 1e-4f && c->state_t > 0.05f)
      set_state(c, ST_ZEROING);
    break;

  case ST_SYSID:
    lo = -X_DOWN_LIMIT_M;
    hi = X_DOWN_LIMIT_M;
    c->sysid_tt += dt;
    if (c->sysid_sub == 0) {
      float xp, vp;
      ctrl_sysid_profile(c->sysid_trial, c->sysid_tt, &xp, &vp);
      a_lim = MAX_CART_ACCEL;
      v_target = vp + 3.0f * (xp - c->x);
      if (c->sysid_tt >= ctrl_sysid_duration(c->sysid_trial)) {
        int n = (c->sysid_k + SYSID_LOG_DECIM - 1) / SYSID_LOG_DECIM;
        emit(out, EV_SYSID_TRIAL_DONE, (float)c->sysid_trial, (float)n, 0);
        out->sysid_trial_done = true;
        c->sysid_sub = 1;
        c->sysid_tt = 0.0f;
        c->settle_t = 0.0f;
      }
    } else {
      a_cmd = down_accel(c);
      a_lim = DOWN_MAX_ACCEL;
      v_target = clampf(c->v + a_cmd * dt, -DOWN_MAX_VELOCITY, DOWN_MAX_VELOCITY);
      c->settle_t = is_settled_down(c) ? c->settle_t + dt : 0.0f;
      bool calm = c->settle_t >= SETTLE_TIME_S || c->sysid_tt >= SYSID_DAMP_TIMEOUT_S;
      if (calm && c->sysid_tt > 1.0f && in->sysid_buffer_free && !out->sysid_trial_done) {
        c->sysid_trial++;
        if (c->sysid_trial > SYSID_NUM_TRIALS) {
          emit(out, EV_SYSID_ALL_DONE, 0, 0, 0);
          c->sysid_trial = 0;
          set_state(c, ST_ZEROING);
        } else {
          emit(out, EV_SYSID_NEXT, (float)c->sysid_trial, 0, 0);
          c->sysid_sub = 0;
          c->sysid_tt = 0.0f;
          c->sysid_k = 0;
        }
      }
    }
    break;

  default:
    v_target = 0.0f;
    break;
  }

  // ---- 6. Безпека: гальмівний шлях до меж, стопери, межі швидкості ----
  if (shape)
    v_target = shape_velocity(v_target, c->x, lo, hi, 0.8f * a_lim);
  v_target = clampf(v_target, -MAX_CART_VELOCITY, MAX_CART_VELOCITY);
  const float v_step_max = (float)MAX_STEP_FREQ_HZ / STEPS_PER_METER;
  v_target = clampf(v_target, -v_step_max, v_step_max);

  float dv = clampf(v_target - c->v, -a_lim * dt, a_lim * dt);
  float v_new = c->v + dv;
  if ((in->stop_left && v_new < 0.0f) || (in->stop_right && v_new > 0.0f))
    v_new = 0.0f; // фізично впираємося у стопер
  c->a = (v_new - c->v) / dt;
  c->v = v_new;
  out->v_cmd = c->v;

  // ---- 7. Запис SysID ----
  if (c->state == ST_SYSID && c->sysid_sub == 0) {
    if (c->sysid_k % SYSID_LOG_DECIM == 0) {
      out->sysid_sample = true;
      out->sysid_trial = c->sysid_trial;
      out->sysid_index = c->sysid_k / SYSID_LOG_DECIM;
      out->sysid_x = c->x;
      out->sysid_v = c->v;
      out->sysid_ticks = in->enc_ticks - c->enc_zero;
    }
    c->sysid_k++;
  }

  // ---- 8. Автокорекція нуля енкодера ----
  // Якщо маятник нерухомий, а візок стоїть — він висить точно внизу (гравітація).
  bool idle = c->state == ST_ZEROING || c->state == ST_WAITING ||
              (c->state == ST_SYSID && c->sysid_sub == 1);
  if (idle && fabsf(c->v) < 0.003f && fabsf(c->phi) < REST_MAX_ANGLE_RAD) {
    if (c->rest_t == 0.0f) {
      c->rest_min = c->rest_max = in->enc_ticks;
    }
    if (in->enc_ticks < c->rest_min)
      c->rest_min = in->enc_ticks;
    if (in->enc_ticks > c->rest_max)
      c->rest_max = in->enc_ticks;
    if (c->rest_max - c->rest_min > REST_MAX_TICKS) {
      c->rest_t = 0.0f;
    } else {
      c->rest_t += dt;
      if (c->rest_t >= REST_TIME_S) {
        int32_t new_zero = c->rest_min + (c->rest_max - c->rest_min + 1) / 2;
        int32_t delta = (new_zero - c->enc_zero) % (int32_t)CPR;
        if (delta > (int32_t)CPR / 2)
          delta -= (int32_t)CPR;
        if (delta < -(int32_t)CPR / 2)
          delta += (int32_t)CPR;
        c->enc_zero = new_zero;
        c->ang_est = (float)(in->enc_ticks - new_zero) * RAD_PER_TICK;
        c->phi = wrap_pi(c->ang_est);
        c->theta = wrap_pi(c->ang_est + PI_F);
        if (delta > 3 || delta < -3)
          emit(out, EV_ZERO_CORRECTED, (float)delta * DEG_PER_TICK, 0, 0);
        c->rest_t = 0.0f;
      }
    }
  } else {
    c->rest_t = 0.0f;
  }
}

// -----------------------------------------------------------------------------
// Текстові назви
// -----------------------------------------------------------------------------
const char *ctrl_state_name(ctrl_state_t s) {
  switch (s) {
  case ST_HOMING_SEEK:
    return "ПОШУК НУЛЯ";
  case ST_HOMING_CENTER:
    return "ВИЇЗД У ЦЕНТР";
  case ST_ZEROING:
    return "ЗАНУЛЕННЯ ВНИЗУ";
  case ST_WAITING:
    return "ОЧІКУЄ 's'";
  case ST_SWINGUP:
    return "ПІДЙОМ";
  case ST_BALANCING:
    return "БАЛАНС";
  case ST_STOPPING:
    return "ЗУПИНКА";
  case ST_SYSID:
    return "SYSID";
  default:
    return "?";
  }
}

int ctrl_event_format(const ctrl_event_t *e, char *buf, int n) {
  switch (e->code) {
  case EV_BOOT:
    return snprintf(buf, n, "Старт ядра керування (%d Гц). Маятник має висіти спокійно.",
                    CTRL_RATE_HZ);
  case EV_HOMED:
    return snprintf(buf, n, "Стопер знайдено: X=%+.4f м (корекція %+.1f мм). Виїзд у центр...",
                    e->a, e->b);
  case EV_HOMING_NO_STOPPER:
    return snprintf(buf, n, "УВАГА: стопер не знайдено! Поточне місце прийнято за X=0.");
  case EV_CENTERED:
    return snprintf(buf, n, "Візок у центрі. Гасіння коливань...");
  case EV_SETTLED:
    return snprintf(buf, n, "Маятник спокійний внизу. Очікую команду [s] / [i].");
  case EV_DISTURBED:
    return snprintf(buf, n, "Порушення спокою (кут %+.1f°, w %+.2f). Гасіння...", e->a, e->b);
  case EV_SWINGUP_START:
    return snprintf(buf, n, "СТАРТ ПІДЙОМУ (траєкторія %.1f с + TVLQR)", e->a);
  case EV_SWINGUP_ABORT:
    if (e->a == 1.0f)
      return snprintf(buf, n, "Підйом скасовано: X=%+.3f м за межею (t=%.2f с)", e->c, e->b);
    return snprintf(buf, n, "Підйом скасовано: відхилення від траєкторії %.0f° (t=%.2f с)",
                    e->c, e->b);
  case EV_SWINGUP_END:
    return snprintf(buf, n, "Траєкторія скінчилась без захоплення (θ=%+.1f°, w=%+.2f)", e->a,
                    e->b);
  case EV_CATCH:
    return snprintf(buf, n, "ВЕРХ ЗЛОВЛЕНО: БАЛАНС (θ=%+.1f°, w=%+.2f, X=%+.3f)", e->a, e->b,
                    e->c);
  case EV_FALL:
    return snprintf(buf, n, "Маятник упав (θ=%+.1f°, X=%+.3f). Зупинка...", e->a, e->b);
  case EV_SOFT_LIMIT:
    return snprintf(buf, n, "Програмна межа рейки X=%+.3f м. Зупинка...", e->a);
  case EV_STOPPER:
    return snprintf(buf, n, "Контакт зі стопером %s! Корекція X %+.1f мм (втрачені кроки?)",
                    e->a < 0 ? "ЛІВИМ" : "ПРАВИМ", e->b);
  case EV_ZERO_CORRECTED:
    return snprintf(buf, n, "Нуль енкодера скориговано на %+.1f° (маятник у спокої = низ)",
                    e->a);
  case EV_SYSID_START:
    return snprintf(buf, n, "СТАРТ SysID (%d тріали)", SYSID_NUM_TRIALS);
  case EV_SYSID_TRIAL_DONE:
    return snprintf(buf, n, "SysID тріал %d завершено (%d зразків)", (int)e->a, (int)e->b);
  case EV_SYSID_NEXT:
    return snprintf(buf, n, "SysID: старт тріалу %d", (int)e->a);
  case EV_SYSID_ALL_DONE:
    return snprintf(buf, n, "SysID: усі тріали завершено");
  case EV_CMD_REJECTED:
    return snprintf(buf, n, "Команду відхилено (код %d) у стані '%s'", (int)e->a,
                    ctrl_state_name((ctrl_state_t)(int)e->b));
  case EV_MANUAL_CENTER:
    return snprintf(buf, n, "Ручне встановлення центру (стара X=%+.3f м)", e->a);
  case EV_ABORT_CMD:
    return snprintf(buf, n, "СТОП за командою (стан '%s'). Плавна зупинка...",
                    ctrl_state_name((ctrl_state_t)(int)e->a));
  default:
    return snprintf(buf, n, "Подія %d", e->code);
  }
}
