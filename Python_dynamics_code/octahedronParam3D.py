import numpy as np
from RigidityMatrix3D import RigidityMatrix3D

b = 30.0
k = 2000.0
m = 1.8
g = 9.81
g_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -m*g, 0.0, 0.0, -m*g, 0.0, 0.0, -m*g])

# Define the initial conditions
RM = RigidityMatrix3D()
x = RM.x
x1 = x[0, 0]
y1 = x[0, 1]
z1 = x[0, 2]
x2 = x[1, 0]
y2 = x[1, 1]
z2 = x[1, 2]
x3 = x[2, 0]
y3 = x[2, 1]
z3 = x[2, 2]
x4 = x[3, 0]
y4 = x[3, 1]
z4 = x[3, 2]
x5 = x[4, 0]
y5 = x[4, 1]
z5 = x[4, 2]
x6 = x[5, 0]
y6 = x[5, 1]
z6 = x[5, 2]

x1_dot = y1_dot = z1_dot = x2_dot = y2_dot = z2_dot = x3_dot = y3_dot = z3_dot = x4_dot = y4_dot = z4_dot = x5_dot = y5_dot = z5_dot = x6_dot = y6_dot = z6_dot = 0.0

# Define the time parameters
Ts = 0.01
t_end = 2.0
n_steps = int(t_end/Ts)
t_start = 0.0

if __name__ == "__main__":
    # Run the model_1_sim file
    import octahedron_1_sim.py
