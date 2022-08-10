from problem.ProblemStatement import Problem
from numerical.DifferentialEquation import solve_linear_differential_equation
from numerical.Interpolation import interpolate
from numerical.DefiniteIntegral import integrate
from numpy import linspace
from coptpy import Envr, Model, tuplelist, quicksum, COPT, LinExpr


def add_control_vars(M: Model, r: int, N: int, L1: list, L2: list):
    u_indices = tuplelist()
    lb = {}
    ub = {}
    for j in range(r):
        for k in range(N):
            u_indices.append((j, k))
            lb[j, k] = L1[j]
            ub[j, k] = L2[j]
    u = M.addVars(u_indices, vtype=COPT.CONTINUOUS, lb=lb, ub=ub, nameprefix='u')

    return u


def add_terminal_constraints(M: Model, A, B, C, H, x0, g, T, N, r, m, u):
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
        zb = solve_linear_differential_equation(-A, B[:, i], reverse_grid)
        spline_zb = interpolate(reverse_grid, zb)
        for k in range(N):
            # Выражение int_(Tk/N)^(T(k+1)/N) (exp(A(T-t))b_i) dt
            D[i, k] = integrate(spline_zb, T*k/N, T*(k+1)/N)
            a_new_ik = H.dot(D[i, k])
            for l in range(m):
                a_new[l][i, k] = a_new_ik[l]

    M.addConstrs((u.prod(a_new[l]) == h[l] for l in range(m)), nameprefix='terminal_constraint')

    return D


def add_objective(M: Model, R, d, c, L1, L2, r, N, T, D, u):
    # Выражение int_0^T(u(t)'*R*u(t))dt
    quadratic_u = quicksum(T/N * R[i1, i2] * u[i1, k] * u[i2, k]
                           for i1 in range(r) for i2 in range(r)
                           for k in range(N) if abs(R[i1, i2]) > 1e-15)

    # Выражение int_0^T(d*|u(t)|)dt
    abs_u = LinExpr(0)
    v = {}
    for i in range(r):
        if abs(d[i]) > 1e-15:
            if L1[i] > -1e-15:
                abs_u += T/N * d[i] * u.sum(i, '*')
            elif L2[i] < 1e-15:
                abs_u += -T/N * d[i] * u.sum(i, '*')
            else:
                for k in range(N):
                    v[i, k, 1] = M.addVar(vtype=COPT.CONTINUOUS, lb=0, ub=-L1[i], name='v[{}, {}, 1]'.format(i, k))
                    v[i, k, 2] = M.addVar(vtype=COPT.CONTINUOUS, lb=0, ub=L2[i], name='v[{}, {}, 2]'.format(i, k))
                    M.addConstr(u[i, k] == v[i, k, 2] - v[i, k, 1], 'abs_split_constraint_{}_{}'.format(i, k))
                abs_u += T/N * d[i] * quicksum((v[i, k, 1] + v[i, k, 2]) for k in range(N))

    # Выражение c*x(T) (без свободного члена)
    cD = {key: c.dot(val) for key, val in D.items()}
    cx = u.prod(cD)

    # Задание целевой функции
    M.setObjective(quadratic_u + abs_u + cx, sense=COPT.MINIMIZE)


def search_piecewise_constant_control(P: Problem):
    env = Envr()
    M: Model = env.createModel('Optimal Piecewise Constant Control Searching')

    u = add_control_vars(M, P.r, P.N, P.L1, P.L2)

    D = add_terminal_constraints(M, P.A, P.B, P.C, P.H, P.x0, P.g, P.T, P.N, P.r, P.m, u)

    add_objective(M, P.R, P.d, P.c, P.L1, P.L2, P.r, P.N, P.T, D, u)

    M.solve()

    print(M.getObjective().getValue())

    return {key: val.X for key, val in u.items()}
