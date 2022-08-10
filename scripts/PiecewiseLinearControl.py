from problem.ProblemStatement import Problem
from numerical.DifferentialEquation import solve_linear_differential_equation
from numerical.Interpolation import interpolate
from numerical.DefiniteIntegral import integrate
from numpy import linspace, array
from coptpy import Envr, Model, tuplelist, tupledict, COPT, QuadExpr


def add_control_coefficients_vars(M: Model, r: int, N: int):
    p_indices = tuplelist(tuplelist((i, k, o) for i in range(r) for k in range(N) for o in range(2)))
    p = M.addVars(p_indices, lb=-COPT.INFINITY, vtype=COPT.CONTINUOUS, nameprefix='p')

    return p


def add_smoothness_constraints(M: Model, r: int, N: int, T: float, p: tupledict):
    for k in range(1, N):
        t = T * k / N
        for i in range(r):
            M.addConstr(p[i, k-1, 1] * t + p[i, k-1, 0] == p[i, k, 1] * t + p[i, k, 0],
                        name='continuity_constraint_{}_{}'.format(i, k))


def add_straight_constraints(M: Model, L1, L2, r: int, N: int, T: float, p: tupledict):
    for k in range(N):
        t1 = T * k / N
        t2 = T * (k + 1) / N
        for i in range(r):
            M.addConstr(p[i, k, 1] * t1 + p[i, k, 0] >= L1[i], name='left_lower_constraint_{}_{}'.format(i, k))
            M.addConstr(p[i, k, 1] * t1 + p[i, k, 0] <= L2[i], name='left_upper_constraint_{}_{}'.format(i, k))
            M.addConstr(p[i, k, 1] * t2 + p[i, k, 0] >= L1[i], name='right_lower_constraint_{}_{}'.format(i, k))
            M.addConstr(p[i, k, 1] * t2 + p[i, k, 0] <= L2[i], name='right_upper_constraint_{}_{}'.format(i, k))


def add_terminal_constraints(M: Model, A, B, C, H, x0, g, T, N, r, m, p):
    grid_size = 501
    straight_grid = linspace(0, T, grid_size)
    reverse_grid = linspace(T, 0, grid_size)

    z_0 = solve_linear_differential_equation(A, x0, straight_grid)

    zc = solve_linear_differential_equation(-A, C, reverse_grid)
    spline_zc = interpolate(reverse_grid, zc)
    int_zc = integrate(spline_zc, 0, T)

    # Решение уравнения при нулевом управлении: dx/dt = Ax + C в точке T
    xT = z_0[-1, :] + int_zc
    h = g - H.dot(xT)

    a_new = {l: {} for l in range(m)}
    D = {}
    for i in range(r):
        zb = [solve_linear_differential_equation(-A, B[:, i], reverse_grid)]
        for o in range(2):
            zb.append(array([zb[o][t] * reverse_grid[t] for t in range(grid_size)]))

        spline_zb = []
        for o in range(2):
            spline_zb.append(interpolate(reverse_grid, zb[o]))
            for k in range(N):
                # Выражение int_(Tk/N)^(T(k+1)/N) (exp(A(T-t))b_i) dt
                D[i, k, o] = integrate(spline_zb[o], T * k / N, T * (k + 1) / N)
                a_new_ik = H.dot(D[i, k, o])
                for l in range(m):
                    a_new[l][i, k, o] = a_new_ik[l]

    M.addConstrs((p.prod(a_new[l]) == h[l] for l in range(m)), nameprefix='terminal_constraint')

    return D


def add_objective(M: Model, R, c, r, N, T, D, p):
    # Выражение int_0^T(u(t)'*R*u(t))dt
    quadratic_u = QuadExpr(0)
    for k in range(N):
        t1 = T * k / N
        t2 = T * (k + 1) / N
        for i1 in range(r):
            for i2 in range(r):
                if abs(R[i1, i2]) <= 1e-15:
                    continue
                expr = ((t2 ** 3 - t1 ** 3) / 3 * p[i1, k, 1] * p[i2, k, 1] +
                        (t2 ** 2 - t1 ** 2) / 2 * (p[i1, k, 0] * p[i2, k, 1] + p[i1, k, 1] * p[i2, k, 0]) +
                        (t2 ** 1 - t1 ** 1) / 1 * p[i1, k, 0] * p[i1, k, 0])
                quadratic_u += R[i1, i2] * expr

    # Выражение c*x(T) (без свободного члена)
    cD = {key: c.dot(val) for key, val in D.items()}
    cx = p.prod(cD)

    # Задание целевой функции
    M.setObjective(quadratic_u + cx, sense=COPT.MINIMIZE)


def search_piecewise_linear_control(P: Problem):
    env = Envr()
    M: Model = env.createModel('Optimal Piecewise Linear Control Searching')

    p = add_control_coefficients_vars(M, P.r, P.N)

    add_smoothness_constraints(M, P.r, P.N, P.T, p)
    add_straight_constraints(M, P.L1, P.L2, P.r, P.N, P.T, p)
    D = add_terminal_constraints(M, P.A, P.B, P.C, P.H, P.x0, P.g, P.T, P.N, P.r, P.m, p)

    add_objective(M, P.R, P.c, P.r, P.N, P.T, D, p)

    M.solve()

    print(M.getObjective().getValue())

    return {key: val.X for key, val in p.items()}
