from typing import Tuple
from numpy import array


class Problem(object):

    def __init__(self) -> None:
        """
        Задание параметров задачи
        """
        # Задание уравнения dx/dt = Ax + Bu + C
        self.A = array([
            [0., 1., 0., 0.],
            [1., 0., 0., -3.],
            [0., 0., 0., 1.],
            [0., 3., 1, 0.]
        ])
        self.B = array([
            [0., 0.],
            [1., 0.],
            [0., 0.],
            [0., 1.]
        ])
        self.C = array([0., 0., 0., 0.])

        # Прямые ограничения на управление L1 <= u <= L2
        self.L1 = array([-2., -2.])
        self.L2 = ([2., 2])

        # Задание терминальных условий x(0) = x0, Hx(T) = g
        self.x0 = array([-10., 5., 6., 8.])
        self.H = array([
            [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.],
        ])
        self.g = array([0., 0., 0., 0.])

        # Целевая функция в виде
        # int_0^T (x(t)'*Q*x(t) + u(t)'*R*u(t) + d*|u(t)|) dt + c*x(T) --> min
        # Сейчас квадратичная форма x(t)'*Q*x(t) не учитывается в ЦФ
        # Член с модулем d*|u(t)| не учитывается в управлении сплайнами
        self.Q = array([
            [0., 0., 0., 0.],
            [0., 0., 0., 0.],
            [0., 0., 0., 0.],
            [0., 0., 0., 0.]
        ])
        self.R = array([
            [1., 0.],
            [0., 1.]
        ])
        self.d = array([0., 0.])
        self.c = array([0., 0., 0., 0.])

        # Время окончания управления
        self.T = 20

        # Количество разбиений отрезка управления
        self.N = 50

        # Определение размерностей задачи
        self.n, self.r, self.m = self.get_dimensions()
        self.check_dimensions()

    def get_dimensions(self) -> Tuple[int, int, int]:
        """
        Определение размерностей задачи
        :return: 1) n --- размерность фазового вектора,
                 2) r --- размерность вектора управлений,
                 3) m --- количество ограничений на фазовый вектор в конечный момент времени
        """
        n = len(self.A)
        r = len(self.L1)
        m = len(self.H)

        return n, r, m

    def check_dimensions(self) -> None:
        """
        Проверка соответствия размерностей параметров задачи
        """
        assert self.n > 0, 'n = 0'
        assert self.r > 0, 'r = 0'

        assert len(self.A[0]) == self.n, \
            'Dimension of A is ({})x({})'.format(self.n, len(self.A[0]))
        assert len(self.B) == self.n, \
            'Number of rows of B is {} instead of {})'.format(len(self.B), self.n)
        assert len(self.B[0]) == self.r, \
            'Number of columns of B is {} instead of {})'.format(len(self.B[0]), self.r)
        assert len(self.C) == self.n, \
            'Dimension of C is {} instead of {})'.format(len(self.C), self.n)

        assert len(self.L2) == self.r, \
            'Dimension of L2 is {} instead of {})'.format(len(self.L2), self.r)

        assert len(self.x0) == self.n, \
            'Dimension of x0 is {} instead of {})'.format(len(self.x0), self.n)
        if self.m > 0:
            assert len(self.H[0]) == self.n, \
                'Number of columns of H is {} instead of {})'.format(len(self.H[0]), self.n)
        assert len(self.g) == self.m, \
            'Dimension of g is {} instead of {})'.format(len(self.g), self.m)

        assert len(self.Q) == self.n, \
            'Number of rows of Q is {} instead of {})'.format(len(self.Q), self.n)
        assert len(self.Q[0]) == self.n, \
            'Number of columns of Q is {} instead of {})'.format(len(self.Q[0]), self.n)
        assert len(self.R) == self.r, \
            'Number of rows of R is {} instead of {})'.format(len(self.R), self.r)
        assert len(self.R[0]) == self.r, \
            'Number of columns of R is {} instead of {})'.format(len(self.R[0]), self.r)
        assert len(self.d) == self.r, \
            'Dimension of d is {} instead of {})'.format(len(self.d), self.r)
        assert len(self.c) == self.n, \
            'Dimension of c is {} instead of {})'.format(len(self.c), self.n)

        print('All dimensions are correct')
