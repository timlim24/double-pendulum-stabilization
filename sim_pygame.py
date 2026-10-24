"""
sim_pygame.py
=============
Точка входу для симуляції одинарного маятника на візку.
Перенаправляє до повної реалізації у single_pendulum/sim_pygame.py.
"""

import sys
from pathlib import Path

# Додаємо каталог single_pendulum до шляхів пошуку модулів
single_pendulum_dir = Path(__file__).resolve().parent / "single_pendulum"
if str(single_pendulum_dir) not in sys.path:
    sys.path.insert(0, str(single_pendulum_dir))

from sim_pygame import RealtimeCartPoleSim

if __name__ == "__main__":
    sim = RealtimeCartPoleSim()
    sim.run()
