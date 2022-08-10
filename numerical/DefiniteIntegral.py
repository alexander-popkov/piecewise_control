from scipy.integrate import quad
from numpy import array


def integrate(func_array, t_start: float, t_finish: float):
    int_array = []
    for j in range(len(func_array)):
        I, err = quad(func_array[j], t_start, t_finish)
        int_array.append(I)

    return array(int_array)
