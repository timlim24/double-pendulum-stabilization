# single_pendulum — цифровий двійник стенду

Тут немає «своєї» логіки керування: усе, що керує, — це `common/controller.c`
(та сама прошивка, що в ESP32), викликана через ctypes (`controller_lib.py`).
Бібліотека збирається gcc автоматично при першому імпорті.

## Модель стенду (`common/host/plant_sim.c`)

Кроковий візок **кінематичний** — він виконує задану швидкість, сила і маса
візка ролі не грають. Тому модель:

```
φ'' = −ω0² sinφ − γ a cosφ − d φ' − c tanh(φ'/ε)      φ — від низу
γ = κ ω0²/g
швидкість візка відстає від команди з постійною часу τ
```

плюс те, що є на залізі: мертва зона генератора кроків (`MIN_STEP_FREQ`),
квантування енкодера (2400 тіків), дебаунс стоперів, фізичні стінки за
стоперами (кроки «губляться»), опційно люфт паса.

Параметри ω0², κ, d, c, τ вимірює `../sys_id.py` і записує в
`common/pendulum_params.json`.

## Файли

| Файл | Призначення |
|---|---|
| `main.py`, `sim_pygame.py` | інтерактивний симулятор (Pygame) |
| `simulation_manager.py` | прошивка + модель стенду в реальному часі |
| `controller_lib.py` | ctypes-обгортка: `Controller`, `Plant`, `Bench` |
| `params.py` | JSON-параметри, LQR, генерація `pendulum_params.h` |
| `hwconfig.py` | константи з `cart_pole_config.h`, enum-и з `controller.h` |
| `mpc_generator.py` | траєкторія підйому (CasADi/IPOPT) + TVLQR → `swingup_trajectory.h` |
| `selftest.py` | автоперевірка прошивки: 9 варіантів розбіжності моделі |
| `virtual_stand.py` | віртуальний стенд на псевдотерміналі (для `cli_control.py`, `sys_id.py`) |

## Команди

```bash
python single_pendulum/sim_pygame.py           # симулятор
python single_pendulum/selftest.py             # автотест (≈3 с), --quick — лише номінал
python single_pendulum/params.py               # перегенерувати pendulum_params.h
python single_pendulum/mpc_generator.py        # перерахувати траєкторію підйому
python single_pendulum/mpc_generator.py --tvlqr-only   # лише коефіцієнти TVLQR
python single_pendulum/virtual_stand.py --speed 4      # віртуальний стенд, ×4 час
```

Клавіші симулятора: `S` підйом, `X`/пробіл стоп, `H` хомінг, `C` центр,
`I` SysID, `←/→` поштовх маятника, `M` розбіжність моделі (ω0² ×0.95/×1.05),
`R` перезапуск.

## Межі робастності (перевірено `selftest.py`)

Підйом проходить при похибці ω0² до ±5% (ширше вже ні — траєкторія
розрахована під конкретний період гойдань), κ ±15%, τ до 12 мс, сухому
й ×3 в'язкому тертю, люфті 1 мм. Тому після зміни маятника (інший вантаж,
довжина) **спершу SysID**, потім підйом.
