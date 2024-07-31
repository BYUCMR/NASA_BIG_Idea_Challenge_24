import matplotlib.pyplot as plt
import numpy as np
import triangleParam2D as P
from triangle_dynamics_2D import TriangleDynamics
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from RigidityMatrix2D import RigidityMatrix2D
from signal_generator import signalGenerator

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
plt.title("Triangle Animation")
plt.xlabel("x")
plt.ylabel("y")

# Create lists to hold the time and node positions
time = []
node1_x = []
node1_y = []
node2_x = []
node2_y = []
node3_x = []
node3_y = []

# Initialize the triangle dynamics object
triangle = TriangleDynamics()
# Initialize the rigidity matrix object, used to initialize the node positions and edges
RM = RigidityMatrix2D()

# Initialize the disturbance input
disturbance = signalGenerator(amplitude = 200.0, frequency = 0.25, y_offset = 0.0)

# Initialization function
def init():
    line1.set_data([RM.x[0,0], RM.x[1,0]], [RM.x[0,1], RM.x[1,1]])
    line2.set_data([RM.x[1,0], RM.x[2,0]], [RM.x[1,1], RM.x[2,1]])
    line3.set_data([RM.x[2,0], RM.x[0,0]], [RM.x[2,1], RM.x[0,1]])
    node1.set_data([float(RM.x[0,0])], [float(RM.x[0,1])])
    node2.set_data([float(RM.x[1,0])], [float(RM.x[1,1])])
    node3.set_data([float(RM.x[2,0])], [float(RM.x[2,1])])

    global node_texts
    node_texts = [
        ax.text(float(RM.x[0,0]), float(RM.x[0,1]), '1', color='black', fontsize=12, ha='right', va='bottom'),
        ax.text(float(RM.x[1,0]), float(RM.x[1,1]), '2', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[2,0]), float(RM.x[2,1]), '3', color='black', fontsize=12, ha='left', va='bottom')
    ]

    return line1, line2, line3, node1, node2, node3, *node_texts

# Update function for animation
def update(frame):

    # Update the time stamp and reset the input forces
    t = frame*P.Ts
    tau = np.zeros((6))

    # Apply the disturbance to the y component of the last node
    # Recall the order is x1, x2, x3, y1, y2, y3
    tau[5] = disturbance.square(t)

    # Update the triangle dynamics
    triangle.update(tau)

    # Get the new node positions
    x, y = triangle.h()

    # Append the time and node positions to the lists
    time.append(t)
    node1_x.append(x[0])
    node1_y.append(y[0])
    node2_x.append(x[1])
    node2_y.append(y[1])
    node3_x.append(x[2])
    node3_y.append(y[2])
    
    # Update the animation
    line1.set_data([x[0], x[1]], [y[0], y[1]])
    line2.set_data([x[1], x[2]], [y[1], y[2]])
    line3.set_data([x[2], x[0]], [y[2], y[0]])
    node1.set_data(x[0], y[0])
    node2.set_data(x[1], y[1])
    node3.set_data(x[2], y[2])

    # Update the text for each node
    for idx, text in enumerate(node_texts):
        text.set_position((x[idx], y[idx]))

    return line1, line2, line3, node1, node2, node3, *node_texts

def save_animation(dynamic_animation):
    f = r"C:/Users/stowel22/Desktop/animation.mp4"
    FFMpegWriter = animation.writers['ffmpeg']
    writer = FFMpegWriter(fps=30, metadata=dict(artist='Me'), bitrate=1800)
    dynamic_animation.save(f, writer=writer)

dynamic_animation = FuncAnimation(fig, update, frames=P.n_steps, init_func=init, blit=True, interval=1000*P.Ts)

# Save the animation (may need to change the 'interval' parameter in dynamic_animation to be 1 to get good frame rate)
# save_animation(dynamic_animation)
plt.show()

# Plot the x and y data for each node
fig, ax = plt.subplots(3, 2)
ax[0, 0].set_title("X")
ax[0, 1].set_title("Y")
ax[0, 0].set_ylabel("Node 1")
ax[1, 0].set_ylabel("Node 2")
ax[2, 0].set_ylabel("Node 3")
ax[0,0].plot(time, node1_x)
ax[0,1].plot(time, node1_y)
ax[1,0].plot(time, node2_x)
ax[1,1].plot(time, node2_y)
ax[2,0].plot(time, node3_x)
ax[2,1].plot(time, node3_y)
plt.show()