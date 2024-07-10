import matplotlib.pyplot as plt
import numpy as np
import triangleParam2D as P
from triangle_dynamics_2D import TriangleDynamics
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from RigidityMatrix2D import RigidityMatrix2D

# import os
# import imageio_ffmpeg as ffmpeg
# os.environ['PATH'] += os.pathsep + ffmpeg.get_ffmpeg_exe()

# TODO: Add subplot that shows the positions of the three nodes wrt time
fig, ax = plt.subplots()
line1, = ax.plot([], [], 'bo-', lw=2)   # Tube from node 1-2
line2, = ax.plot([], [], 'bo-', lw=2)   # Tube from node 2-3
line3, = ax.plot([], [], 'bo-', lw=2)   # Tube from node 3-1
node1, = ax.plot([], [], 'ro', lw=2)    # Node 1
node2, = ax.plot([], [], 'yo', lw=2)    # Node 2
node3, = ax.plot([], [], 'go', lw=2)    # Node 3
ax.set_aspect('equal')
ax.set_xlim(-0.5, 2)
ax.set_ylim(-0.5, 2)
ax.grid()
plt.title("Truss Animation")
plt.xlabel("x")
plt.ylabel("y")

# Create a second figure that plots the x y coordinates of each node in time
fig2, ax2 = plt.subplots(3, 2)

# Initialize the triangle dynamics object
triangle = TriangleDynamics()
# Initialize the rigidity matrix object, used to initialize the node positions and edges
RM = RigidityMatrix2D()

# Initialization function
def init():
    line1.set_data([RM.x[0,0], RM.x[1,0]], [RM.x[0,1], RM.x[1,1]])
    line2.set_data([RM.x[1,0], RM.x[2,0]], [RM.x[1,1], RM.x[2,1]])
    line3.set_data([RM.x[2,0], RM.x[0,0]], [RM.x[2,1], RM.x[0,1]])
    node1.set_data([float(RM.x[0,0])], [float(RM.x[0,1])])
    node2.set_data([float(RM.x[1,0])], [float(RM.x[1,1])])
    node3.set_data([float(RM.x[2,0])], [float(RM.x[2,1])])
    return line1, line2, line3, node1, node2, node3

# Update function for animation
def update(frame):
    if frame < 150:
        # Follows the order: x1, x2, x3, y1, y2, y3
        tau = np.array([0.0, 0.0, 0.0, 0.0, 0.0, P.m*P.g*10.0])
    else:
        tau = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    triangle.update(tau)
    x, y = triangle.h()
    # print(x, y)
    line1.set_data([x[0], x[1]], [y[0], y[1]])
    line2.set_data([x[1], x[2]], [y[1], y[2]])
    line3.set_data([x[2], x[0]], [y[2], y[0]])
    node1.set_data(x[0], y[0])
    node2.set_data(x[1], y[1])
    node3.set_data(x[2], y[2])
    return line1, line2, line3, node1, node2, node3

def save_animation(dynamic_animation):
    f = r"C:/Users/stowel22/Desktop/animation.mp4"
    FFMpegWriter = animation.writers['ffmpeg']
    writer = FFMpegWriter(fps=30, metadata=dict(artist='Me'), bitrate=1800)
    dynamic_animation.save(f, writer=writer)

dynamic_animation = FuncAnimation(fig, update, frames=P.n_steps, init_func=init, blit=True, interval=1000*P.Ts)

# Save the animation (may need to change the 'interval' parameter in dynamic_animation to be 1 to get good frame rate)
# save_animation(dynamic_animation)
plt.show()