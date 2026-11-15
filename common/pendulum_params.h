// АВТОГЕНЕРОВАНО single_pendulum/params.py з common/pendulum_params.json
// НЕ РЕДАГУВАТИ ВРУЧНУ. Перегенерувати: python single_pendulum/params.py
#pragma once

// Модель: θ'' = ω0² sinθ + γ a cosθ − d θ' − c tanh(θ'/ε), θ від верху
#define PEND_OMEGA0_SQ 34.497754f
#define PEND_OMEGA0 5.873479f
#define PEND_GAMMA 3.708321f
#define PEND_D_VISC 0.013884f
#define PEND_C_COUL 0.005491f
#define PEND_TAU_ACT 0.002419f

// LQR балансу: a = -(K_X x + K_TH θ + K_V v + K_W ω + K_I ∫x)
// Q = diag(40.0, 80.0, 6.0, 2.0, 20.0), R = 0.5
#define LQR_K_X -14.76582f
#define LQR_K_TH 50.13274f
#define LQR_K_V -11.39342f
#define LQR_K_W 8.48996f
#define LQR_K_I -6.19729f

// Гасіння внизу: a = DOWN_K_DAMP · ω · cosφ  (ζ = 0.35)
#define DOWN_K_DAMP 1.10871f
