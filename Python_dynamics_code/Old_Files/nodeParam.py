import numpy as np

b = 5.0
k = 200.0*10
m = 3.0
g = 9.81
theta = np.pi/4

# Define the initial conditions
x1 = 0.0
y1 = 0.0
x1_dot = 0.0
y1_dot = 0.0

# Define the time parameters
Ts = 0.01
t_end = 3.0
n_steps = int(t_end / Ts)
t_start = 0.0

ell = 2.0

if __name__ == "__main__":
    # run the model_1_sim file
    import model_1_sim