import matplotlib.pyplot as plt
import numpy as np
import octahedronParam3D as P
from octahedron_dynamics_3D import OctahedronDynamics
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from RigidityMatrix3D import RigidityMatrix3D
from mpl_toolkits.mplot3d import Axes3D

# import os
# import imageio_ffmpeg as ffmpeg
# os.environ['PATH'] += os.pathsep + ffmpeg.get_ffmpeg_exe()

# TODO: Add subplot that shows the positions of the nodes wrt time
fig, ax = plt.subplots(subplot_kw = {'projection': '3d'})
line1, = ax.plot([], [], [], 'bo-', lw=2) 
line2, = ax.plot([], [], [], 'bo-', lw=2)
line3, = ax.plot([], [], [], 'bo-', lw=2)
line4, = ax.plot([], [], [], 'bo-', lw=2)
line5, = ax.plot([], [], [], 'bo-', lw=2)
line6, = ax.plot([], [], [], 'bo-', lw=2)
line7, = ax.plot([], [], [], 'bo-', lw=2)
line8, = ax.plot([], [], [], 'bo-', lw=2)
line9, = ax.plot([], [], [], 'bo-', lw=2)
line10, = ax.plot([], [], [], 'bo-', lw=2)
line11, = ax.plot([], [], [], 'bo-', lw=2)
line12, = ax.plot([], [], [], 'bo-', lw=2)
node1, = ax.plot([], [], [], 'ro', lw=2)
node2, = ax.plot([], [], [], 'ro', lw=2)
node3, = ax.plot([], [], [], 'ro', lw=2)
node4, = ax.plot([], [], [], 'ro', lw=2)
node5, = ax.plot([], [], [], 'ro', lw=2)
node6, = ax.plot([], [], [], 'ro', lw=2)

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)

plt.title("Octahedron Animation")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

time = []
node1_x = []
node1_y = []
node1_z = []
node2_x = []
node2_y = []
node2_z = []
node3_x = []
node3_y = []
node3_z = []
node4_x = []
node4_y = []
node4_z = []
node5_x = []
node5_y = []
node5_z = []
node6_x = []
node6_y = []
node6_z = []

# Create a second picture that plots the x, y and z coordinates of each node in time
# fig2, ax2 = plt.subplots(3, 2)

# Initialize the octahedron dynamics object
octahedron = OctahedronDynamics()
# Initialize the rigidity matrix object, used to initialize the node positions and edges
RM = RigidityMatrix3D()

# Initializtion function
def init():
    line1.set_data([RM.x[0, 0], RM.x[1, 0]], [RM.x[0, 1], RM.x[1, 1]]) # 1-2
    line1.set_3d_properties([RM.x[0, 2], RM.x[1, 2]])
    line2.set_data([RM.x[1, 1], RM.x[2, 1]], [RM.x[1, 0], RM.x[2, 0]]) # 3-2
    line2.set_3d_properties([RM.x[2, 2], RM.x[1, 2]])
    line3.set_data([RM.x[2, 1], RM.x[0, 1]], [RM.x[2, 0], RM.x[0, 0]]) # 3-1
    line3.set_3d_properties([RM.x[0, 2], RM.x[2, 2]])
    line4.set_data([RM.x[4, 0], RM.x[3, 0]], [RM.x[4, 1], RM.x[3, 1]]) # 5-4
    line4.set_3d_properties([RM.x[4, 2], RM.x[3, 2]])
    line5.set_data([RM.x[3, 0], RM.x[2, 0]], [RM.x[3, 1], RM.x[2, 1]]) # 4-3
    line5.set_3d_properties([RM.x[3, 2], RM.x[2, 2]])
    line6.set_data([RM.x[2, 0], RM.x[4, 0]], [RM.x[2, 1], RM.x[4, 1]]) # 3-5
    line6.set_3d_properties([RM.x[2, 2], RM.x[4, 2]])
    line7.set_data([RM.x[4, 0], RM.x[1, 0]], [RM.x[4, 1], RM.x[1, 1]]) # 5-2
    line7.set_3d_properties([RM.x[4, 2], RM.x[1, 2]])
    line8.set_data([RM.x[1, 0], RM.x[5, 0]], [RM.x[1, 1], RM.x[5, 1]]) # 2-6
    line8.set_3d_properties([RM.x[1, 2], RM.x[5, 2]])
    line9.set_data([RM.x[5, 0], RM.x[4, 0]], [RM.x[5, 1], RM.x[4, 1]]) # 6-5
    line9.set_3d_properties([RM.x[5, 2], RM.x[4, 2]])
    line10.set_data([RM.x[0, 0], RM.x[3, 0]], [RM.x[0, 1], RM.x[3, 1]]) # 1-4
    line10.set_3d_properties([RM.x[0, 2], RM.x[3, 2]])
    line11.set_data([RM.x[3, 0], RM.x[5, 0]], [RM.x[3, 1], RM.x[5, 1]]) # 4-6
    line11.set_3d_properties([RM.x[3, 2], RM.x[5, 2]])
    line12.set_data([RM.x[5, 0], RM.x[0, 0]], [RM.x[5, 1], RM.x[0, 1]]) # 6-1
    line12.set_3d_properties([RM.x[5, 2], RM.x[0, 2]])
    node1.set_data([RM.x[0, 0]], [RM.x[0, 1]])
    node1.set_3d_properties([RM.x[0, 2]])
    node2.set_data([RM.x[1, 0]], [RM.x[1, 1]])
    node2.set_3d_properties(RM.x[1, 2])
    node3.set_data([RM.x[2, 0]], [RM.x[2, 1]])
    node3.set_3d_properties(RM.x[2, 2])
    node4.set_data([RM.x[3, 0]], [RM.x[3, 1]])
    node4.set_3d_properties(RM.x[3, 2])
    node5.set_data([RM.x[4, 0]], [RM.x[4, 1]])
    node5.set_3d_properties(RM.x[4, 2])
    node6.set_data([RM.x[5, 0]], [RM.x[5, 1]])
    node6.set_3d_properties(RM.x[5, 2])
    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, node1, node2, node3, node4, node5, node6

# Update function for animation
def update(frame):
    if frame < 150:
        tau = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [P.g*P.m], [0.0], [0.0], [P.g*P.m], [0.0], [0.0], [P.g*P.m]])
    else:
        tau = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
    octahedron.update(tau)
    x, y, z = octahedron.h()
    

    line1.set_data([float(x[0]), float(x[1])], [float(y[0]), float(y[1])]) #1-2
    line1.set_3d_properties([float(z[0]), float(z[1])])
    line2.set_data([float(x[2]), float(x[1])], [float(y[2]), float(y[1])]) #2-3
    line2.set_3d_properties([float(z[2]), float(z[1])])
    line3.set_data([float(x[0]), float(x[2])], [float(y[0]), float(y[2])]) #3-1
    line3.set_3d_properties([float(z[0]), float(z[2])])
    line4.set_data([float(x[4]), float(x[3])], [float(y[4]), float(y[3])]) #5-4
    line4.set_3d_properties([float(z[4]), float(z[3])])
    line5.set_data([float(x[3]), float(x[2])], [float(y[3]), float(y[2])]) #4-3
    line5.set_3d_properties([float(z[3]), float(z[2])])
    line6.set_data([float(x[2]), float(x[4])], [float(y[2]), float(y[4])]) #3-5
    line6.set_3d_properties([float(z[2]), float(z[4])])
    line7.set_data([float(x[4]), float(x[1])], [float(y[4]), float(y[1])]) #5-2
    line7.set_3d_properties([float(z[4]), float(z[1])])
    line8.set_data([float(x[1]), float(x[5])], [float(y[1]), float(y[5])]) #2-6
    line8.set_3d_properties([float(z[1]), float(z[5])])
    line9.set_data([float(x[5]), float(x[4])], [float(y[5]), float(y[4])]) #6-5
    line9.set_3d_properties([float(z[5]), float(z[4])])
    line10.set_data([float(x[0]), float(x[3])], [float(y[0]), float(y[3])]) #1-4
    line10.set_3d_properties([float(z[0]), float(z[3])])
    line11.set_data([float(x[3]), float(x[5])], [float(y[3]), float(y[5])]) #4-6
    line11.set_3d_properties([float(z[3]), float(z[5])])
    line12.set_data([float(x[5]), float(x[0])], [float(y[5]), float(y[0])]) #6-1
    line12.set_3d_properties([float(z[5]), float(z[0])])
    node1.set_data(x[0], y[0])
    node1.set_3d_properties(z[0])
    node2.set_data(x[1], y[1])
    node2.set_3d_properties(z[1])
    node3.set_data(x[2], y[2])
    node3.set_3d_properties(z[2])
    node4.set_data(x[3], y[3])
    node4.set_3d_properties(z[3])
    node5.set_data(x[4], y[4])
    node5.set_3d_properties(z[4])
    node6.set_data(x[5], y[5])
    node6.set_3d_properties(z[5])

    time.append(frame*P.Ts)
    node1_x.append(x[0])
    node1_y.append(y[0])
    node1_z.append(z[0])
    node2_x.append(x[1])
    node2_y.append(y[1])
    node2_z.append(z[1])
    node3_x.append(x[2])
    node3_y.append(y[2])
    node3_z.append(z[2])
    node4_x.append(x[3])
    node4_y.append(y[3])
    node4_z.append(z[3])
    node5_x.append(x[4])
    node5_y.append(y[4])
    node5_z.append(z[4])
    node6_x.append(x[5])
    node6_y.append(y[5])
    node6_z.append(z[5])
    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, node1, node2, node3, node4, node5, node6

dynamic_animation = FuncAnimation(fig, update, frames = P.n_steps, init_func=init, blit=True, interval=10000*P.Ts)
# init()
plt.show()

# Create a second figure that plots the x, y and z coordinates of each node in time
fig2, axs = plt.subplots(6, 3, figsize = (10, 8))
axs[0, 0].set_title("X")
axs[0, 1].set_title("Y")
axs[0, 2].set_title("Z")
for i in range(6):
    for j in range(3):
        axs[i, j].set_xlim(0, P.n_steps*P.Ts)
        axs[i, j].set_ylim(-0.5, 2)

axs[0, 0].plot(time, node1_x)
axs[0, 1].plot(time, node1_y)
axs[0, 2].plot(time, node1_z)
axs[1, 0].plot(time, node2_x)
axs[1, 1].plot(time, node2_y)
axs[1, 2].plot(time, node2_z)
axs[2, 0].plot(time, node3_x)
axs[2, 1].plot(time, node3_y)
axs[2, 2].plot(time, node3_z)
axs[3, 0].plot(time, node4_x)
axs[3, 1].plot(time, node4_y)
axs[3, 2].plot(time, node4_z)
axs[4, 0].plot(time, node5_x)
axs[4, 1].plot(time, node5_y)
axs[4, 2].plot(time, node5_z)
axs[5, 0].plot(time, node6_x)
axs[5, 1].plot(time, node6_y)
axs[5, 2].plot(time, node6_z)

plt.show()