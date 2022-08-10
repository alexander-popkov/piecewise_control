from numpy import array
from scipy.integrate import odeint


def linear_equation(x, t, A: array):
    return A.dot(x)


def solve_linear_differential_equation(A, x0, t_grid):
    x = odeint(linear_equation, x0, t_grid, args=(A,))

    return x
