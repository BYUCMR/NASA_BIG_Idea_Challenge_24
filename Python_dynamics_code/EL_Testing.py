import numpy as np
import matplotlib.pyplot as plt
import sympy as sp

# from sympy import init_printing

# # this is responsible to visualize time derivatives with dots
# init_printing()

# define the variables
k, b, m, g, theta, F, t = sp.symbols('k b m g theta F t')

# Define the generalized coordinates
x1, y1 = sp.Function('x1')(t), sp.Function('y1')(t)
Q = sp.Matrix([[x1], [y1]])

# Define the position of the body in terms of the generalized coordinates
P = sp.Matrix([[x1],[y1],[0]])

# Define the velocity of the body in terms of the generalized coordinates
V = sp.diff(P, t)

# Define the kinetic and potential energy
K = 1 / 2 * m * V.dot(V)

# Define the potential energy
U = m * g * (sp.cos(theta) + sp.sin(theta)) + 1/2 * k * (x1**2 + y1**2)

# Define the lagrangian
L = K - U

# Assuming the lagrangian is correct:
# sp.pprint(V)
EL = sp.diff(L.diff(Q.diff(t)), t) - L.diff(Q)
EL = EL.simplify()
EL = sp.pprint(EL)

Q_dot = Q.diff(t)
sp.pprint(Q)
Beta = sp.Matrix([[b],[b]])

Tau = sp.Matrix([[F*sp.cos(theta)],[F*sp.sin(theta)]])

sol = EL + Q_dot @ Beta - Tau

sol_separated = (sp.solve(sol, Q_dot.diff(t)))

sp.pprint(sol_separated)