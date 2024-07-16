import numpy as np
from solar_array_rigidity_matrix import SolarRigidityMatrix

b = 50.0
k = 2000.0
m = 1.8
g = 9.81

# 9 nodes with 3 DOF each provides 27 indices
g_vector = np.zeros((27))
# The last 9 indices are the z direction of each node
g_vector[18:27] = -g*m

# Define the initial conditions
RM = SolarRigidityMatrix()
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
x7 = x[6, 0]
y7 = x[6, 1]
z7 = x[6, 2]
x8 = x[7, 0]
y8 = x[7, 1]
z8 = x[7, 2]
x9 = x[8, 0]
y9 = x[8, 1]
z9 = x[8, 2]

x1_dot = y1_dot = z1_dot = x2_dot = y2_dot = z2_dot = x3_dot = y3_dot = z3_dot = x4_dot = y4_dot = z4_dot = x5_dot = y5_dot = z5_dot = x6_dot = y6_dot = z6_dot = 0.0
x7_dot = y7_dot = z7_dot = x8_dot = y8_dot = z8_dot = x9_dot = y9_dot = z9_dot = 0.0

# Define the time parameters
Ts = 0.01
t_end = 10.0
n_steps = int(t_end/Ts)
t_start = 0.0

if __name__ == "__main__":
    # Run the model_1_sim file
    import solar_array_sim
