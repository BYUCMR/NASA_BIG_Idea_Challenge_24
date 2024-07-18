import numpy as np
from crane_rigidity_matrix import CraneRigidityMatrix

b = 50.0
k = 3000.0
m = 1.8
g = 9.81/6.0

# 30 nodes with 3 DOF each provides 90 indices
g_vector = np.zeros((90))
# The last 30 indices are the z direction of each node
g_vector[60:90] = -g*m

# Define the initial conditions
RM = CraneRigidityMatrix()
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
x10 = x[9, 0]
y10 = x[9, 1]
z10 = x[9, 2]
x11 = x[10, 0]
y11 = x[10, 1]
z11 = x[10, 2]
x12 = x[11, 0]
y12 = x[11, 1]
z12 = x[11, 2]
x13 = x[12, 0]
y13 = x[12, 1]
z13 = x[12, 2]
x14 = x[13, 0]
y14 = x[13, 1]
z14 = x[13, 2]
x15 = x[14, 0]
y15 = x[14, 1]
z15 = x[14, 2]
x16 = x[15, 0]
y16 = x[15, 1]
z16 = x[15, 2]
x17 = x[16, 0]
y17 = x[16, 1]
z17 = x[16, 2]
x18 = x[17, 0]
y18 = x[17, 1]
z18 = x[17, 2]
x19 = x[18, 0]
y19 = x[18, 1]
z19 = x[18, 2]
x20 = x[19, 0]
y20 = x[19, 1]
z20 = x[19, 2]
x21 = x[20, 0]
y21 = x[20, 1]
z21 = x[20, 2]
x22 = x[21, 0]
y22 = x[21, 1]
z22 = x[21, 2]
x23 = x[22, 0]
y23 = x[22, 1]
z23 = x[22, 2]
x24 = x[23, 0]
y24 = x[23, 1]
z24 = x[23, 2]
x25 = x[24, 0]
y25 = x[24, 1]
z25 = x[24, 2]
x26 = x[25, 0]
y26 = x[25, 1]
z26 = x[25, 2]
x27 = x[26, 0]
y27 = x[26, 1]
z27 = x[26, 2]
x28 = x[27, 0]
y28 = x[27, 1]
z28 = x[27, 2]
x29 = x[28, 0]
y29 = x[28, 1]
z29 = x[28, 2]
x30 = x[29, 0]
y30 = x[29, 1]
z30 = x[29, 2]

# Unless specifically desired otherwise, all initial velocities (x1_dot & so forth) are set to zero in the dynamics code
x1_dot = y1_dot = z1_dot = x2_dot = y2_dot = z2_dot = x3_dot = y3_dot = z3_dot = x4_dot = y4_dot = z4_dot = x5_dot = y5_dot = z5_dot = x6_dot = y6_dot = z6_dot = 0.0
x7_dot = y7_dot = z7_dot = x8_dot = y8_dot = z8_dot = x9_dot = y9_dot = z9_dot = x10_dot = y10_dot = z10_dot = x11_dot = y11_dot = z11_dot = x12_dot = y12_dot = z12_dot = 0.0
x13_dot = y13_dot = z13_dot = x14_dot = y14_dot = z14_dot = x15_dot = y15_dot = z15_dot = x16_dot = y16_dot = z16_dot = x17_dot = y17_dot = z17_dot = x18_dot = y18_dot = z18_dot = 0.0
x19_dot = y19_dot = z19_dot = x20_dot = y20_dot = z20_dot = x21_dot = y21_dot = z21_dot = x22_dot = y22_dot = z22_dot = x23_dot = y23_dot = z23_dot = x24_dot = y24_dot = z24_dot = 0.0
x25_dot = y25_dot = z25_dot = x26_dot = y26_dot = z26_dot = x27_dot = y27_dot = z27_dot = x28_dot = y28_dot = z28_dot = x29_dot = y29_dot = z29_dot = x30_dot = y30_dot = z30_dot = 0.0

# Define the time parameters
Ts = 0.01
t_end = 10.0
n_steps = int(t_end/Ts)
t_start = 0.0

if __name__ == "__main__":
    # Run the model_1_sim file
    import crane_sim
