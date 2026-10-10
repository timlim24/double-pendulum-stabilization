"""
server.py
=========
FastAPI web server with real-time WebSocket telemetry streaming for the
Inverted Double Pendulum on a Cart.

Serves:
  - GET /: Interactive HTML5 simulation visualization and control interface.
  - WebSocket /ws: 60 FPS real-time bi-directional streaming of simulation state.
  - POST /api/push: Triggers physical disturbance impulse on the pendulums.
  - POST /api/reset: Resets simulation state (defaults to DOWN position: theta1=pi, theta2=pi).
  - POST /api/swing_up: Triggers the dynamic swing-up maneuver from down to upright.
  - GET /api/status: Returns controller info, gains, poles, and system parameters.
"""

import asyncio
import json
import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Any, List, Optional

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# Ensure double_pendulum directory is in path
current_dir = Path(__file__).resolve().parent
if str(current_dir) not in sys.path:
    sys.path.insert(0, str(current_dir))

from config import PendulumConfig, DEFAULT_CONFIG
from lqr_solver import LQRController, DoublePendulumSimulator, wrap_angle

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("pendulum_server")

app = FastAPI(
    title="Inverted Double Pendulum LQR Simulator",
    description="Real-time web simulator for the Inverted Double Pendulum on a Cart with Swing-Up and LQR Control",
    version="1.1.0"
)

# Global simulation state & controller
cfg = DEFAULT_CONFIG
controller = LQRController(cfg)
simulator = DoublePendulumSimulator(cfg, controller)

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self._lock:
            self.active_connections.append(websocket)
        logger.info(f"WebSocket client connected. Total clients: {len(self.active_connections)}")

    async def disconnect(self, websocket: WebSocket):
        async with self._lock:
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)
        logger.info(f"WebSocket client disconnected. Total clients: {len(self.active_connections)}")

    async def broadcast(self, message: str):
        async with self._lock:
            connections = list(self.active_connections)
        
        for conn in connections:
            try:
                await conn.send_text(message)
            except Exception:
                async with self._lock:
                    if conn in self.active_connections:
                        self.active_connections.remove(conn)

manager = ConnectionManager()

# Background simulation runner
is_running = True
is_paused = False
simulation_lock = threading.Lock()


def physics_loop():
    """
    High-frequency physics integration loop running in background thread.
    Simulates at 500 Hz (dt = 0.002s).
    """
    global is_running, is_paused
    target_dt = cfg.dt  # 0.002 s = 2 ms
    logger.info("Physics simulation thread started.")

    while is_running:
        t_start = time.perf_counter()
        if not is_paused:
            with simulation_lock:
                simulator.step()

        elapsed = time.perf_counter() - t_start
        sleep_time = target_dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


# Start physics thread on server boot
physics_thread = threading.Thread(target=physics_loop, daemon=True)
physics_thread.start()


# Async broadcaster for WebSockets (streams at 60 Hz)
async def broadcast_loop():
    """Streams simulation state to connected WebSocket clients at stream_fps."""
    broadcast_interval = 1.0 / cfg.stream_fps
    while is_running:
        t_start = time.perf_counter()
        if manager.active_connections:
            with simulation_lock:
                data = simulator.get_telemetry()
            payload = json.dumps(data)
            await manager.broadcast(payload)

        elapsed = time.perf_counter() - t_start
        sleep_time = broadcast_interval - elapsed
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            await asyncio.sleep(0.001)


@app.on_event("startup")
async def startup_event():
    asyncio.create_task(broadcast_loop())
    logger.info(f"Double Pendulum Server started. UI available at http://{cfg.host}:{cfg.port}")


@app.on_event("shutdown")
def shutdown_event():
    global is_running
    is_running = False


# --- API Models ---
class PushRequest(BaseModel):
    direction: float = 1.0       # +1.0 for right, -1.0 for left
    strength: float = 1.0        # Scaling factor for default push torque
    torque1: Optional[float] = None
    torque2: Optional[float] = None
    duration: Optional[float] = None


class ResetRequest(BaseModel):
    start_down: bool = True
    initial_tilt_deg: Optional[float] = None


# --- HTTP Endpoints ---
@app.get("/", response_class=HTMLResponse)
async def get_index():
    """Serves the main visualization user interface index.html."""
    html_path = current_dir / "index.html"
    if not html_path.is_file():
        raise HTTPException(status_code=404, detail="index.html not found")
    return HTMLResponse(content=html_path.read_text(encoding="utf-8"))


@app.post("/api/push")
async def trigger_push(req: PushRequest):
    """
    Applies an impulse disturbance (push) to the pendulum links.
    """
    scale = req.direction * req.strength
    t1 = req.torque1 if req.torque1 is not None else (cfg.default_push_torque1 * scale)
    t2 = req.torque2 if req.torque2 is not None else (cfg.default_push_torque2 * scale)
    dur = req.duration if req.duration is not None else cfg.push_duration

    with simulation_lock:
        simulator.apply_disturbance(torque1=t1, torque2=t2, duration=dur)

    logger.info(f"Disturbance triggered: torque1={t1:.4f} N*m, torque2={t2:.4f} N*m, duration={dur:.3f} s")
    return {"status": "success", "message": "Disturbance applied", "torque1": t1, "torque2": t2, "duration": dur}


@app.post("/api/reset")
async def trigger_reset(req: Optional[ResetRequest] = None):
    """
    Resets the simulation. Defaults to starting from the DOWN position.
    """
    global is_paused
    start_down = True if req is None else req.start_down
    with simulation_lock:
        if req and req.initial_tilt_deg is not None and abs(req.initial_tilt_deg) > 1e-4:
            init_rad = np.radians(req.initial_tilt_deg)
            init_state = np.array([0.0, init_rad, -0.7 * init_rad, 0.0, 0.0, 0.0], dtype=np.float64)
            simulator.reset(init_state, start_down=False)
        else:
            simulator.reset(start_down=start_down)

    logger.info(f"Simulation reset. start_down={start_down}")
    return {"status": "success", "message": f"Simulation reset. start_down={start_down}"}


@app.post("/api/swing_up")
async def trigger_swing_up():
    """Triggers the swing-up maneuver from current state."""
    with simulation_lock:
        simulator.trigger_swing_up()
    return {"status": "success", "message": "Swing-up initiated"}


@app.post("/api/pause")
async def toggle_pause():
    """Toggles pause/resume of the simulation."""
    global is_paused
    is_paused = not is_paused
    return {"status": "success", "paused": is_paused}


@app.get("/api/status")
async def get_status():
    """Returns system parameters, LQR matrices, and controller diagnostics."""
    with simulation_lock:
        telemetry = simulator.get_telemetry()

    return {
        "telemetry": telemetry,
        "config": {
            "M": cfg.M,
            "m1": cfg.m1,
            "m2": cfg.m2,
            "L1": cfg.L1,
            "L2": cfg.L2,
            "a1": cfg.a1,
            "a2": cfg.a2,
            "I1": cfg.I1,
            "I2": cfg.I2,
            "k0": cfg.k0,
            "k1": cfg.k1,
            "k2": cfg.k2,
            "g": cfg.g,
            "F_max": cfg.F_max,
            "X_max": cfg.X_max,
            "dt": cfg.dt,
            "start_from_down": cfg.start_from_down,
            "swing_up_duration": cfg.swing_up_duration,
        },
        "lqr": {
            "q_x": cfg.q_x,
            "q_th1": cfg.q_th1,
            "q_th2": cfg.q_th2,
            "q_dx": cfg.q_dx,
            "q_dth1": cfg.q_dth1,
            "q_dth2": cfg.q_dth2,
            "r_u": cfg.r_u,
            "K": [round(float(k), 4) for k in controller.K],
            "is_controllable": controller.is_controllable,
            "closed_loop_poles": [
                {"real": round(float(p.real), 4), "imag": round(float(p.imag), 4)}
                for p in controller.closed_loop_poles
            ],
        }
    }


# --- WebSocket Endpoint ---
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    Bi-directional WebSocket streaming endpoint.
    Streams telemetry at 60 Hz and accepts incoming commands from the client.
    """
    await manager.connect(websocket)
    try:
        while True:
            raw_msg = await websocket.receive_text()
            try:
                cmd = json.loads(raw_msg)
                action = cmd.get("action")

                if action == "push":
                    dir_val = float(cmd.get("direction", 1.0))
                    strength = float(cmd.get("strength", 1.0))
                    scale = dir_val * strength
                    t1 = cfg.default_push_torque1 * scale
                    t2 = cfg.default_push_torque2 * scale
                    dur = cfg.push_duration
                    with simulation_lock:
                        simulator.apply_disturbance(torque1=t1, torque2=t2, duration=dur)

                elif action == "reset":
                    start_down = bool(cmd.get("start_down", True))
                    tilt = cmd.get("tilt_deg", None)
                    with simulation_lock:
                        if tilt is not None and abs(float(tilt)) > 1e-4:
                            rad = np.radians(float(tilt))
                            simulator.reset(np.array([0.0, rad, -0.7 * rad, 0.0, 0.0, 0.0]), start_down=False)
                        else:
                            simulator.reset(start_down=start_down)

                elif action == "swing_up":
                    with simulation_lock:
                        simulator.trigger_swing_up()

                elif action == "pause":
                    global is_paused
                    is_paused = not is_paused

            except Exception as parse_err:
                logger.warning(f"Failed to process WebSocket message: {parse_err}")

    except WebSocketDisconnect:
        await manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        await manager.disconnect(websocket)


if __name__ == "__main__":
    uvicorn.run("server:app", host=cfg.host, port=cfg.port, reload=False, log_level="info")
