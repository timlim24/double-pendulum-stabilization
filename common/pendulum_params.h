// АВТОГЕНЕРОВАНО single_pendulum/params.py з common/pendulum_params.json
// НЕ РЕДАГУВАТИ ВРУЧНУ. Перегенерувати: python single_pendulum/params.py
#pragma once

// Модель: θ'' = ω0² sinθ + γ a cosθ − d θ' − c tanh(θ'/ε), θ від верху
#define PEND_OMEGA0_SQ 35.040000f
#define PEND_OMEGA0 5.919459f
#define PEND_GAMMA 3.571865f
#define PEND_D_VISC 0.092000f
#define PEND_C_COUL 0.000000f
#define PEND_TAU_ACT 0.002000f

// LQR балансу: a = -(K_X x + K_TH θ + K_V v + K_W ω + K_I ∫x)
// Q = diag(40.0, 80.0, 6.0, 2.0, 20.0), R = 0.5
#define LQR_K_X -14.73633f
#define LQR_K_TH 52.35419f
#define LQR_K_V -11.31929f
#define LQR_K_W 8.74868f
#define LQR_K_I -6.19832f

// Гасіння внизу: a = DOWN_K_DAMP · ω · cosφ  (ζ = 0.35)
#define DOWN_K_DAMP 1.16007f
