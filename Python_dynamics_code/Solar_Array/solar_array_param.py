import numpy as np
from solar_array_rigidity_matrix import SolarRigidityMatrix

b = 50.0
k = 2000.0
m = 1.975
g = 9.81

# 9 nodes with 3 DOF each provides 27 indices
g_vector = np.zeros((27))
# The last 9 indices are the z direction of each node
g_vector[18:27] = -g*m

# All initial conditions are defined in the dynamics file
# RM = SolarRigidityMatrix()
# x = RM.x

# Define the time parameters
Ts = 0.01
t_end = 10.0
n_steps = int(t_end/Ts)
t_start = 0.0

if __name__ == "__main__":
    # Run the model_1_sim file
    import solar_array_sim
