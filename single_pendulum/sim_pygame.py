"""
sim_pygame.py
=============
Точка входу для запуску симулятора одинарного перевернутого маятника на візку.
"""

from single_pendulum.main import SinglePendulumApp

if __name__ == "__main__":
    app = SinglePendulumApp()
    app.run()
