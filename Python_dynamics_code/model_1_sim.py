import matplotlib.pyplot as plt
import numpy as np
import trussParam as P
from model_1_dynamics import TrussDynamics
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
line1, = ax.plot([], [], 'bo-', lw=2)
line2, = ax.plot([], [], 'bo-', lw=2)
anchor1, = ax.plot([], [], 'ro', lw=2)  # Anchor for spring-damper 1
anchor2, = ax.plot([], [], 'ro', lw=2)  # Anchor for spring-damper 2
ax.set_xlim(-1, 4)
ax.set_ylim(-1, 4)
ax.grid()

truss = TrussDynamics()

# Initialization function
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    anchor1.set_data([P.ell, 0])
    anchor2.set_data([0.0, P.ell])
    return line1, line2, anchor1, anchor2

# Update function for animation
def update(frame):
    F = 100.0
    truss.update(F)
    x, y = truss.h()
    # print(x, y)
    line1.set_data([x[0], P.ell], [y[0], 0.0])
    line2.set_data([x[0], 0.0], [y[0], P.ell])
    return line1, line2, anchor1, anchor2

animation = FuncAnimation(fig, update, frames=P.n_steps, init_func=init, blit=True, interval=P.Ts*1000)
plt.title("Truss Animation")
plt.xlabel("x")
plt.ylabel("y")
plt.show()