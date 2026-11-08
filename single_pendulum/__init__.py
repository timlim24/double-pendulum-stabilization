"""
single_pendulum
===============
Цифровий двійник стенду одинарного маятника на кроковому візку.

  hwconfig.py          — константи з common/cart_pole_config.h та enum-и з controller.h
  params.py            — ідентифіковані параметри (common/pendulum_params.json) і LQR
  controller_lib.py    — ctypes-обгортка над C-ядром прошивки та моделлю стенду
  mpc_generator.py     — оптимальна траєкторія підйому + TVLQR -> swingup_trajectory.h
  simulation_manager.py, main.py, ui.py — інтерактивний симулятор (Pygame)
  selftest.py          — автоперевірка прошивки в симуляції
"""
