def run() -> None:
    """
    Запуск поиска решения задачи оптимального управления
    """
    from problem.ProblemStatement import Problem
    from control.PiecewiseConstantControl import search_piecewise_constant_control
    from control.PiecewiseLinearControl import search_piecewise_linear_control
    from control.QuadraticSplineControl import search_quadratic_spline_control
    from movement.PCCObjectMovement import calculate_pcc_movement
    from movement.PLCObjectMovement import calculate_plc_movement
    from movement.QSCObjectMovement import calculate_qsc_movement
    from plotting.Plotting import plot
    from datetime import datetime

    mode = 'quadratic'
    P = Problem()
    now = datetime.now()
    if mode == 'constant':
        u = search_piecewise_constant_control(P)
        t_grid, x_grid, u_grid = calculate_pcc_movement(P, u)
    elif mode == 'linear':
        p = search_piecewise_linear_control(P)
        t_grid, x_grid, u_grid = calculate_plc_movement(P, p)
    elif mode == 'quadratic':
        p = search_quadratic_spline_control(P)
        t_grid, x_grid, u_grid = calculate_qsc_movement(P, p)
    else:
        exit(1)
    print((datetime.now() - now).total_seconds())
    plot(t_grid, x_grid, u_grid)


if __name__ == '__main__':
    run()
