import do_mpc


def template_simulator(model, integration_tool='collocation'):
    """
    Налаштування симулятора для DAE моделі подвійного перевернутого маятника.

    Параметри:
        model: DAE модель do-mpc
        integration_tool: інструмент інтегрування ('collocation' або 'idas').
            'collocation' (ортогональна колокація) забезпечує ідеальну чисельну
            стійкість для систем DAE високого порядку без збоїв у кроках інтегрування.
    """
    simulator = do_mpc.simulator.Simulator(model)

    params_simulator = {
        'integration_tool': integration_tool,
        't_step': 0.04
    }
    if integration_tool == 'idas':
        params_simulator.update({
            'abstol': 1e-5,
            'reltol': 1e-5,
            'integration_opts': {
                'suppress_algebraic': True
            }
        })

    simulator.set_param(**params_simulator)
    simulator.setup()

    return simulator


if __name__ == '__main__':
    from template_model import template_model
    model = template_model()
    sim = template_simulator(model)
    print("Симулятор для DIP успішно налаштований!")

