#!/usr/bin/env bash
# Quickstart launcher for the Inverted Double Pendulum Simulator

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Activate virtualenv if present
if [ -d "$REPO_DIR/.venv" ]; then
    source "$REPO_DIR/.venv/bin/activate"
fi

echo "=========================================================="
echo " Starting Inverted Double Pendulum LQR Simulation Suite  "
echo " Open your browser at: http://localhost:8000             "
echo " Press Ctrl+C to stop.                                   "
echo "=========================================================="

python3 "$SCRIPT_DIR/server.py"

