import do_mpc


def template_mpc(model, silence_solver=True):
    """
    Налаштування MPC контролера для підйому (swing-up) та стабілізації DIP.

    Параметри:
        model: DAE модель do-mpc
        silence_solver: якщо True, вимикає надлишковий вивід IPOPT на кожній ітерації
    """
    mpc = do_mpc.controller.MPC(model)

    nlpsol_opts = {
        'ipopt.linear_solver': 'mumps'
    }
    if silence_solver:
        nlpsol_opts.update({
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        })

    setup_mpc = {
        'n_horizon': 100,           # Горизонт прогнозування (100 кроків)
        'n_robust': 0,
        'open_loop': 0,
        't_step': 0.04,             # Крок за часом (0.04 с -> горизонт 4.0 с)
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 3,
        'collocation_ni': 1,
        'store_full_solution': True,
        'nlpsol_opts': nlpsol_opts
    }
    mpc.set_param(**setup_mpc)

    # Цільова функція: максимізація потенційної та мінімізація кінетичної енергії
    # E_kin - E_pot (підйом обох ланок у вертикальне положення без фазових проблем 2*pi)
    mterm = model.aux['E_kin'] - model.aux['E_pot']  # Термінальні витрати
    lterm = model.aux['E_kin'] - model.aux['E_pot']  # Крокові витрати

    mpc.set_objective(mterm=mterm, lterm=lterm)

    # Штраф за швидкість зміни керування
    mpc.set_rterm(force=0.1)

    # Обмеження на керуючу силу: [-4, 4] Н
    mpc.bounds['lower', '_u', 'force'] = -4.0
    mpc.bounds['upper', '_u', 'force'] = 4.0

    mpc.setup()

    return mpc


if __name__ == '__main__':
    from template_model import template_model
    model = template_model()
    mpc = template_mpc(model)
    print("MPC контролер для DIP успішно налаштований!")

