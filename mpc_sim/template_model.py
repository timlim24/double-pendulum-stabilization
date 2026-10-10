import numpy as np
from casadi import *
import do_mpc


def template_model():
    """
    Побудова моделі подвійного перевернутого маятника на рухомому візку (DIP)
    у вигляді системи диференціально-алгебраїчних рівнянь (DAE).
    """
    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    # Фізичні параметри
    m0 = 0.6    # кг, маса візка
    m1 = 0.2    # кг, маса першої ланки
    m2 = 0.2    # кг, маса другої ланки
    L1 = 0.5    # м, довжина першої ланки
    L2 = 0.5    # м, довжина другої ланки
    g = 9.80665 # м/с^2, прискорення вільного падіння

    # Похідні геометричні та динамічні параметри
    l1 = L1 / 2.0  # відстань до центру мас ланки 1
    l2 = L2 / 2.0  # відстань до центру мас ланки 2
    J1 = (m1 * l1**2) / 3.0   # момент інерції ланки 1
    J2 = (m2 * l2**2) / 3.0   # момент інерції ланки 2

    h1 = m0 + m1 + m2
    h2 = m1 * l1 + m2 * L1
    h3 = m2 * l2
    h4 = m1 * l1**2 + m2 * L1**2 + J1
    h5 = m2 * l2 * L1
    h6 = m2 * l2**2 + J2
    h7 = (m1 * l1 + m2 * L1) * g
    h8 = m2 * l2 * g

    # Диференціальні змінні стану (x):
    # pos - координата візка
    # theta (2,1) - кути відхилення [theta_1, theta_2] (0 рад = верхнє вертикальне положення)
    # dpos - швидкість візка
    # dtheta (2,1) - кутові швидкості ланок
    pos = model.set_variable('_x', 'pos')
    theta = model.set_variable('_x', 'theta', (2, 1))
    dpos = model.set_variable('_x', 'dpos')
    dtheta = model.set_variable('_x', 'dtheta', (2, 1))

    # Керуюча змінна (u): горизонтальна сила на візок
    u = model.set_variable('_u', 'force')

    # Алгебраїчні змінні (z): другі похідні [ddpos, ddtheta_1, ddtheta_2]
    ddpos = model.set_variable('_z', 'ddpos')
    ddtheta = model.set_variable('_z', 'ddtheta', (2, 1))

    # Праві частини диференціальних рівнянь:
    model.set_rhs('pos', dpos)
    model.set_rhs('theta', dtheta)
    model.set_rhs('dpos', ddpos)
    model.set_rhs('dtheta', ddtheta)

    # Неявний алгебраїчний зв'язок (рівняння Ейлера-Лагранжа: g(x, u, z) = 0):
    euler_lagrange = vertcat(
        # 1. Горизонтальний рух візка
        h1 * ddpos + h2 * ddtheta[0] * cos(theta[0]) + h3 * ddtheta[1] * cos(theta[1])
        - (h2 * dtheta[0]**2 * sin(theta[0]) + h3 * dtheta[1]**2 * sin(theta[1]) + u),
        # 2. Обертання ланки 1
        h2 * cos(theta[0]) * ddpos + h4 * ddtheta[0] + h5 * cos(theta[0] - theta[1]) * ddtheta[1]
        - (h7 * sin(theta[0]) - h5 * dtheta[1]**2 * sin(theta[0] - theta[1])),
        # 3. Обертання ланки 2
        h3 * cos(theta[1]) * ddpos + h5 * cos(theta[0] - theta[1]) * ddtheta[0] + h6 * ddtheta[1]
        - (h5 * dtheta[0]**2 * sin(theta[0] - theta[1]) + h8 * sin(theta[1]))
    )

    model.set_alg('euler_lagrange', euler_lagrange)

    # Допоміжні вирази: Кінетична та потенціальна енергія системи
    E_kin_cart = 0.5 * m0 * dpos**2
    E_kin_p1 = 0.5 * m1 * (
        (dpos + l1 * dtheta[0] * cos(theta[0]))**2 +
        (l1 * dtheta[0] * sin(theta[0]))**2
    ) + 0.5 * J1 * dtheta[0]**2
    E_kin_p2 = 0.5 * m2 * (
        (dpos + L1 * dtheta[0] * cos(theta[0]) + l2 * dtheta[1] * cos(theta[1]))**2 +
        (L1 * dtheta[0] * sin(theta[0]) + l2 * dtheta[1] * sin(theta[1]))**2
    ) + 0.5 * J2 * dtheta[1]**2

    E_kin = E_kin_cart + E_kin_p1 + E_kin_p2
    E_pot = m1 * g * l1 * cos(theta[0]) + m2 * g * (L1 * cos(theta[0]) + l2 * cos(theta[1]))

    model.set_expression('E_kin', E_kin)
    model.set_expression('E_pot', E_pot)

    model.setup()

    return model


if __name__ == '__main__':
    model = template_model()
    print("Модель DIP успішно створена:")
    print(f"Змінні стану x: {model.x.keys()}")
    print(f"Керування u: {model.u.keys()}")
    print(f"Алгебраїчні змінні z: {model.z.keys()}")

