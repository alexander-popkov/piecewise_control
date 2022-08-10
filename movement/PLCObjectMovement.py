from numpy import linspace, array
from scipy.integrate import odeint


def calculate_control(p, T, N, r):
    return lambda t: array([((p[i, int(t*N/T), 1] * t + p[i, int(t*N/T), 0]) if t < T
                             else (p[i, N-1, 1] * t + p[i, N-1, 0])) for i in range(r)])


def closed_model(x, t, A, B, C, func_u):
    return A.dot(x) + B.dot(func_u(t)) + C


def calculate_plc_movement(P, p):
    grid_size = 501
    t_grid = linspace(0, P.T, grid_size)

    func_u = calculate_control(p, P.T, P.N, P.r)
    u_grid = array([func_u(t) for t in t_grid])

    x_grid = odeint(closed_model, P.x0, t_grid, args=(P.A, P.B, P.C, func_u))

    return t_grid, x_grid, u_grid
