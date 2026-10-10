#!/usr/bin/env bash
# Quickstart launcher for the Inverted Double Pendulum Simulator

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Activate virtualenv if present
if [ -f "$REPO_DIR/venv/bin/activate" ]; then
    source "$REPO_DIR/venv/bin/activate"
elif [ -f "$REPO_DIR/.venv/bin/activate" ]; then
    source "$REPO_DIR/.venv/bin/activate"
fi

# Explicit path to virtualenv python to work even with sudo
PYTHON_BIN="python3"
if [ -f "$REPO_DIR/venv/bin/python3" ]; then
    PYTHON_BIN="$REPO_DIR/venv/bin/python3"
elif [ -f "$REPO_DIR/.venv/bin/python3" ]; then
    PYTHON_BIN="$REPO_DIR/.venv/bin/python3"
fi

# Check for a free port (defaults to 8000, falls back to 8001, 8080, etc.)
PORT_TO_USE=$("$PYTHON_BIN" -c "
import socket
for p in [8000, 8001, 8080, 8888, 5000]:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('0.0.0.0', p))
            print(p)
            break
    except OSError:
        pass
")

PORT_TO_USE="${PORT:-${PORT_TO_USE:-8001}}"

echo "=========================================================="
echo " Starting Inverted Double Pendulum LQR Simulation Suite  "
echo " Open your browser at: http://localhost:$PORT_TO_USE"
echo " Press Ctrl+C to stop.                                   "
echo "=========================================================="

exec "$PYTHON_BIN" -m uvicorn server:app --app-dir "$SCRIPT_DIR" --host 0.0.0.0 --port "$PORT_TO_USE"
