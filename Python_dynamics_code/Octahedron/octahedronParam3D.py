import numpy as np
from RigidityMatrix3D import RigidityMatrix3D

b = 55.0
k = 920.0
m = 1.975*2 + 1.0
g = 9.81
g_vector = np.zeros((18))
g_vector[12:18] = -g*m

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
y4 = 0.10870757842194056 - 0.87239199 - 0.011
z4 = x[3, 2]
x5 = x[4, 0]
y5 = 0.08678381165745996 + 0.43670979 - 0.011
z5 = x[4, 2]
x6 = x[5, 0]
y6 = 0.07539931495255825 + 0.43670915 - 0.011
z6 = x[5, 2]

x1_dot = y1_dot = z1_dot = x2_dot = y2_dot = z2_dot = x3_dot = y3_dot = z3_dot = x4_dot = y4_dot = z4_dot = x5_dot = y5_dot = z5_dot = x6_dot = y6_dot = z6_dot = 0.0

# Define the time parameters
Ts = 0.01
t_end = 50.0
n_steps = int(t_end/Ts)
t_start = 0.0

if __name__ == "__main__":
    # Run the model_1_sim file
    import octahedron_1_sim
    import octahedron_data_visualization
