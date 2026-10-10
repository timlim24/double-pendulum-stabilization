# Inverted Double Pendulum on a Cart: LQR Control Suite

A comprehensive control engineering software suite and mathematical documentation for modeling, controlling, and simulating an **Inverted Double Pendulum on a Cart** using a Linear Quadratic Regulator (LQR).

All equations, physical parameters, damping friction terms, and linearized state-space formulations are derived directly from the repository's MATLAB source-of-truth files (`ParameterEstimation/DoublePendulum/`).

---

## 📁 Folder Structure

```
double_pendulum/
├── formulas.md         # Deliverable 1: Complete LaTeX mathematical derivations & EOM
├── config.py           # Deliverable 2: Central configuration & physical parameters
├── lqr_solver.py       # Deliverable 3: CARE Riccati solver, LQR law, RK4 simulator
├── server.py           # Deliverable 4: FastAPI server & WebSocket streaming backend
├── index.html          # Deliverable 4: Interactive HTML5 Canvas GUI & Chart.js graph
├── run.sh              # Quick launch bash script
└── README.md           # This documentation guide
```

---

## 🎯 Control Objective & System Physics

1. **Start from DOWN Position**:
   - The system **starts from the stable downward hanging position** ($\theta_1 = 180^\circ, \theta_2 = 180^\circ$).
   - A dynamic swing-up excitation drives the double pendulum upwards toward the inverted vertical apex.
   - As the links approach the upright vertical zone, the **LQR controller** seamlessly catches and balances the double pendulum.
2. **X-Invariant Upright Stabilization**:
   - The primary control objective is to keep both pendulum links in a **perfectly vertical (upright) position** ($\theta_1 = 0$, $\theta_2 = 0$).
   - The controller ignores the cart's position on the X-axis ($Q_{1,1} = q_x = 0 \implies K_x = 0$).
   - The cart can settle anywhere along the track as long as the pendulums remain vertical.
3. **Physical Constraints**:
   - Link 1 is longer than Link 2: $L_1 = 0.1727\,\text{m} > L_2 = 0.1400\,\text{m}$.
   - Realistic friction: Viscous track friction ($k_0 = 0.5\,\text{N}\cdot\text{s/m}$) and joint damping ($k_1 = 2.37\times 10^{-4}\,\text{N}\cdot\text{m}\cdot\text{s/rad}$, $k_2 = 1.00\times 10^{-5}\,\text{N}\cdot\text{m}\cdot\text{s/rad}$).
   - Actuator saturation: Motor force is limited to $[-F_{\max}, F_{\max}] = [-40\,\text{N}, +40\,\text{N}]$.
   - Track bounds: The cart operates on a physical linear track with hard limits $[-X_{\max}, X_{\max}] = [-0.8\,\text{m}, +0.8\,\text{m}]$.
4. **Disturbance Recovery**:
   - When the pendulum is pushed, the LQR controller rapidly generates counteracting motor force to catch and restore the upright equilibrium.

---

## 🚀 Quickstart Guide

### 1. Prerequisites & Environment
Ensure Python $\ge 3.10$ is used with the virtual environment:
```bash
source .venv/bin/activate
```

Dependencies:
- `numpy`
- `scipy`
- `fastapi`
- `uvicorn`
- `websockets`

### 2. Standalone LQR & Physics Test
Verify controllability, CARE Riccati solution, and dynamic step response:
```bash
python double_pendulum/lqr_solver.py
```

### 3. Launch Web-Based Interactive Simulator
Start the web server:
```bash
python double_pendulum/server.py
```
or run:
```bash
bash double_pendulum/run.sh
```

Then open your browser at:
👉 **[http://localhost:8000](http://localhost:8000)**

---

## 🕹️ Interactive Web Features

- **HTML5 Canvas Animation**:
  - Live 60 FPS rendering of the cart chassis, wheels, and dual pendulum links.
  - Distinct color-coded links ($L_1 > L_2$), bearing hubs, and centers of mass.
  - Real-time motor force vector arrow on the cart showing magnitude and direction.
  - Visual pulse effect during disturbance impulses.
  - Track limits ($\pm X_{\max}$) with boundary bumper zones.
- **Actuator Saturation Graph (Chart.js)**:
  - Real-time scrolling chart of motor effort $u(t)$ [N].
  - Dashed boundary lines at $+F_{\max}$ and $-F_{\max}$.
  - Visual alert badge and trace color change when actuator saturation occurs.
- **Push Pendulum Buttons**:
  - Primary **"PUSH PENDULUM"** button applies an impulse disturbance.
  - Directional "Push Left" and "Push Right" buttons.
  - Adjustable disturbance strength multiplier slider.
- **Keyboard Shortcuts**:
  - `Space`: Push pendulum
  - `Arrow Left` / `Arrow Right`: Push left / right
  - `R`: Reset to upright equilibrium
  - `P`: Pause / Resume simulation

