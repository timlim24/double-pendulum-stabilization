"""
params.py
=========
Єдине джерело правди для ідентифікованих параметрів маятника та коефіцієнтів
регуляторів.

  common/pendulum_params.json  — редагується руками або sys_id.py
  common/pendulum_params.h     — ГЕНЕРУЄТЬСЯ звідси (не редагувати вручну)

Модель (така сама в прошивці, симуляції, генераторі траєкторії та SysID).
Візок кроковий => кінематичний: вхід — прискорення візка a (м/с^2).
θ — кут від ВЕРХНЬОЇ вертикалі у знаковій конвенції енкодера стенду:

    θ'' = ω0² sinθ + γ a cosθ − d θ' − c tanh(θ'/ε)

ω0² = m g l / J,  γ = κ ω0² / g  (для жорсткого маятника γ = m l / J = ω0²/g;
κ — "ефективне підсилення приводу", 1.0 якщо візок точно виконує команду).

Запуск як скрипт перегенеровує common/pendulum_params.h.
"""

import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import scipy.linalg
from scipy.signal import cont2discrete

try:
    from .hwconfig import COMMON_DIR, HW
except ImportError:
    from hwconfig import COMMON_DIR, HW

PARAMS_JSON = COMMON_DIR / "pendulum_params.json"
PARAMS_H = COMMON_DIR / "pendulum_params.h"
G = 9.81
COULOMB_EPS = 0.05  # рад/с, згладжування sign() у сухому терті


@dataclass
class PendulumParams:
    omega0_sq: float = 35.04  # (рад/с)^2
    kappa: float = 1.0  # ефективне підсилення приводу
    d_visc: float = 0.092  # в'язке тертя (1/с)
    c_coul: float = 0.0  # сухе тертя (рад/с^2)
    tau_act: float = 0.002  # запізнення приводу (с), 1-го порядку

    # Ваги LQR балансу: [x, θ, v, ω, ∫x], R
    lqr_q: list = field(default_factory=lambda: [40.0, 80.0, 6.0, 2.0, 20.0])
    lqr_r: float = 0.5
    # Бажане відносне затухання маятника при гасінні внизу
    down_zeta: float = 0.35
    # Метадані останньої ідентифікації
    sysid_info: dict = field(default_factory=dict)

    @property
    def gamma(self) -> float:
        return self.kappa * self.omega0_sq / G

    @property
    def omega0(self) -> float:
        return float(np.sqrt(self.omega0_sq))


def load_params(path: Path = PARAMS_JSON) -> PendulumParams:
    if not Path(path).exists():
        return PendulumParams()
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    known = {k: v for k, v in data.items() if k in PendulumParams.__dataclass_fields__}
    return PendulumParams(**known)


def save_params(p: PendulumParams, path: Path = PARAMS_JSON) -> None:
    Path(path).write_text(
        json.dumps(asdict(p), indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


# -----------------------------------------------------------------------------
# Лінійні моделі
# -----------------------------------------------------------------------------
def linear_top(p: PendulumParams, with_integral: bool):
    """Лінеаризація біля верхньої точки. Стан [x, θ, v, ω(, ∫x)], вхід a."""
    A = np.array(
        [
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, p.omega0_sq, 0, -p.d_visc],
        ],
        dtype=float,
    )
    B = np.array([[0], [0], [1], [p.gamma]], dtype=float)
    if with_integral:
        A = np.block([[A, np.zeros((4, 1))], [np.array([[1, 0, 0, 0, 0]])]])
        B = np.vstack([B, [[0]]])
    return A, B


def dlqr(A, B, Q, R, dt):
    Ad, Bd, *_ = cont2discrete((A, B, np.eye(A.shape[0]), np.zeros((A.shape[0], 1))), dt)
    P = scipy.linalg.solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K.ravel(), P, Ad, Bd


def balance_gains(p: PendulumParams, dt: float = None):
    """K для a = -K·[x, θ, v, ω, ∫x] (дискретний LQR на частоті контуру)."""
    dt = dt or HW["CTRL_DT"]
    A, B = linear_top(p, with_integral=True)
    K, *_ = dlqr(A, B, np.diag(p.lqr_q), np.array([[p.lqr_r]]), dt)
    return K


def terminal_cost_4state(p: PendulumParams, dt: float):
    """P для 4-станового LQR (без інтегратора) — кінцева умова TVLQR підйому."""
    A, B = linear_top(p, with_integral=False)
    K, P, *_ = dlqr(A, B, np.diag(p.lqr_q[:4]), np.array([[p.lqr_r]]), dt)
    return K, P


def down_damp_gain(p: PendulumParams) -> float:
    """a = K ω cosφ дає φ'' + γK φ' + ω0² φ = 0  =>  K = 2 ζ ω0 / γ."""
    return 2.0 * p.down_zeta * p.omega0 / p.gamma


# -----------------------------------------------------------------------------
# Генерація C-заголовка
# -----------------------------------------------------------------------------
def write_c_header(p: PendulumParams, path: Path = PARAMS_H) -> None:
    K = balance_gains(p)
    kd = down_damp_gain(p)
    lines = [
        "// АВТОГЕНЕРОВАНО single_pendulum/params.py з common/pendulum_params.json",
        "// НЕ РЕДАГУВАТИ ВРУЧНУ. Перегенерувати: python single_pendulum/params.py",
        "#pragma once",
        "",
        "// Модель: θ'' = ω0² sinθ + γ a cosθ − d θ' − c tanh(θ'/ε), θ від верху",
        f"#define PEND_OMEGA0_SQ {p.omega0_sq:.6f}f",
        f"#define PEND_OMEGA0 {p.omega0:.6f}f",
        f"#define PEND_GAMMA {p.gamma:.6f}f",
        f"#define PEND_D_VISC {p.d_visc:.6f}f",
        f"#define PEND_C_COUL {p.c_coul:.6f}f",
        f"#define PEND_TAU_ACT {p.tau_act:.6f}f",
        "",
        "// LQR балансу: a = -(K_X x + K_TH θ + K_V v + K_W ω + K_I ∫x)",
        f"// Q = diag{tuple(p.lqr_q)}, R = {p.lqr_r}",
        f"#define LQR_K_X {K[0]:.5f}f",
        f"#define LQR_K_TH {K[1]:.5f}f",
        f"#define LQR_K_V {K[2]:.5f}f",
        f"#define LQR_K_W {K[3]:.5f}f",
        f"#define LQR_K_I {K[4]:.5f}f",
        "",
        f"// Гасіння внизу: a = DOWN_K_DAMP · ω · cosφ  (ζ = {p.down_zeta})",
        f"#define DOWN_K_DAMP {kd:.5f}f",
        "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    params = load_params()
    if not PARAMS_JSON.exists() or "--save" in sys.argv:
        save_params(params)
    write_c_header(params)
    K = balance_gains(params)
    print(f"ω0 = {params.omega0:.3f} рад/с, γ = {params.gamma:.3f}")
    print("LQR K [x, θ, v, ω, ∫x] =", np.round(K, 3))
    print(f"DOWN_K_DAMP = {down_damp_gain(params):.3f}")
    print(f"✓ {PARAMS_H}")
