import numpy as np
from crane_rigidity_matrix import CraneRigidityMatrix

b = 50.0
k = 3000.0
m = 1.97
g = 9.81/6.0

# 30 nodes with 3 DOF each provides 90 indices
g_vector = np.zeros((90))
# The last 30 indices are the z direction of each node
g_vector[60:90] = -g*m

# All initial conditions are defined at the start of the dynamics file using the initial 'x' values from the RM object

# Define the time parameters
Ts = 0.01
t_end = 5.0
n_steps = int(t_end/Ts)
t_start = 0.0

if __name__ == "__main__":
    # Run the model_1_sim file
    import crane_sim
