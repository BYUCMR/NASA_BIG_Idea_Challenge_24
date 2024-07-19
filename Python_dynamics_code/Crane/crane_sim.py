import matplotlib.pyplot as plt
import numpy as np
import crane_param as P
from crane_dynamics import CraneDynamics
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from crane_rigidity_matrix import CraneRigidityMatrix
from mpl_toolkits.mplot3d import Axes3D
from signal_generator import signalGenerator

# import os
# import ffmpeg as ffmpeg
# os.environ['PATH'] += os.pathsep + ffmpeg.get_ffmpeg_exe()

RM = CraneRigidityMatrix()

Edges = RM.Edges
x = RM.x
num_edges = np.size(Edges, 0)

# TODO: Add subplot that shows the positions of the nodes wrt time
fig, ax = plt.subplots(subplot_kw = {'projection': '3d'})

lines = []
for i in range(num_edges):
    color = "#{:06x}".format(np.random.randint(0, 0xFFFFFF))
    line, = ax.plot([], [], [], 'o-', lw=2, color=color)
    lines.append(line)

nodes = []
node_colors = np.array(['ro', 'yo', 'go', 'co', 'bo', 'ko', 'mo', 'wo', 'ro', 'yo', 'go', 'co', 'bo', 'ko', 'mo', 'wo', 'ro', 'yo', 'go', 'co', 'bo', 'ko', 'mo', 'wo', 'ro', 'yo', 'go', 'co', 'bo', 'ko', 'mo'])
num_nodes = int(np.size(x)/3)
for i in range(num_nodes):
    node, = ax.plot([], [], [], node_colors[i], lw=2)
    nodes.append(node)

ax.set_xlim(-3, 4)
ax.set_ylim(-3, 4)
ax.set_zlim(-3, 4)

plt.title("Crane Animation")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

time = []
node_coordinates = np.array([[],[],[]])

# Create a second picture that plots the x, y and z coordinates of each node in time
# fig2, ax2 = plt.subplots(3, 2)

# Initialize the octahedron dynamics object (used to update the dynamics later)
crane = CraneDynamics()

# Initialize the disturbance input
disturbance = signalGenerator(amplitude = 500.0, frequency = 0.4, y_offset = 0.0)

node_texts = [
    ax.text(float(RM.x[i, 0]), float(RM.x[i, 1]), float(RM.x[i, 2]), str(i + 1), color='black', fontsize=12, ha='left', va='bottom')
    for i in range(num_nodes)
]

# Update function for animation
def update(frame):

    tau = np.zeros((90))
    # Select the type of input disturbance here. Options include square, step, sin (sine), random, and sawtooth.
    # parameters defining the input signal are defined on line ~76-ish in this file
    tau[62] = -disturbance.square(frame*P.Ts)

    crane.update(tau)
    x, y, z = crane.h()
    
    Edges = RM.Edges
    # TODO: Update the section below to iteratively update a line array for all edges
    
    for i, line in enumerate(lines):
        edge = Edges[i]
        line.set_data([x[edge[0],0], x[edge[1],0]], [y[edge[0],0], y[edge[1],0]])
        line.set_3d_properties([z[edge[0],0], z[edge[1],0]])

    for i, node in enumerate(nodes):
        node.set_data(x[i,0], y[i,0])
        node.set_3d_properties(z[i,0])

    time.append(frame*P.Ts)

    for coord, axis in zip([x, y, z], node_coordinates):
        for i in range(num_nodes):
            np.append(coord[i, 0], axis)
    
    for idx, text in enumerate(node_texts):
        text.set_position((float(x[idx]), float(y[idx])))
        text.set_3d_properties(float(z[idx]), zdir='x')

def save_animation(dynamic_animation):
    f = r"C:/Users/spenc/Downloads/animation.mp4"
    FFMpegWriter = animation.writers['ffmpeg']
    writer = FFMpegWriter(fps=int(1/P.Ts), metadata=dict(artist='spenc'))
    dynamic_animation.save(f, writer=writer)

dynamic_animation = FuncAnimation(fig, update, frames=P.n_steps, interval=10) # Interval parameter is the time between frames in milliseconds
plt.show()
# print("")
# userInput = input("Save animation? (y/n): ")
# if userInput == 'y':
# save_animation(dynamic_animation)
# TODO: Add a subplot that shows the x, y, and z coordinates of each node in time

# plt.show()