"""
sim_pygame.py
=============
Точка входу симулятора одинарного маятника (цифровий двійник стенду).
    python sim_pygame.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "single_pendulum"))

from main import SinglePendulumApp  # noqa: E402

if __name__ == "__main__":
    SinglePendulumApp().run()
