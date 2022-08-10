from scipy.interpolate import interp1d
from numpy import array


def interpolate(t_grid, x_grid):
    n = x_grid.shape[1]
    spline = array([interp1d(t_grid, x_grid[:, j], kind='cubic') for j in range(n)])

    return spline
