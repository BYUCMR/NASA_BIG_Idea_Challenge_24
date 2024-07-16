import matplotlib.pyplot as plt
import numpy as np
import solar_array_param as P
from solar_array_dynamics import SolarDynamics
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from solar_array_rigidity_matrix import SolarRigidityMatrix
from mpl_toolkits.mplot3d import Axes3D
from signal_generator import signalGenerator

# import os
# import imageio_ffmpeg as ffmpeg
# os.environ['PATH'] += os.pathsep + ffmpeg.get_ffmpeg_exe()

# TODO: Add subplot that shows the positions of the nodes wrt time
fig, ax = plt.subplots(subplot_kw = {'projection': '3d'})
line1, = ax.plot([], [], [], 'o-', lw=2, color='#1f77b4')   # Blue
line2, = ax.plot([], [], [], 'o-', lw=2, color='#ff7f0e')   # Orange
line3, = ax.plot([], [], [], 'o-', lw=2, color='#2ca02c')   # Green
line4, = ax.plot([], [], [], 'o-', lw=2, color='#d62728')   # Red
line5, = ax.plot([], [], [], 'o-', lw=2, color='#9467bd')   # Purple
line6, = ax.plot([], [], [], 'o-', lw=2, color='#8c564b')   # Brown
line7, = ax.plot([], [], [], 'o-', lw=2, color='#e377c2')   # Pink
line8, = ax.plot([], [], [], 'o-', lw=2, color='#7f7f7f')   # Gray
line9, = ax.plot([], [], [], 'o-', lw=2, color='#bcbd22')   # Yellow
line10, = ax.plot([], [], [], 'o-', lw=2, color='#17becf')  # Teal/Cyan
line11, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Salmon
line12, = ax.plot([], [], [], 'o-', lw=2, color='#aec7e8')  # Light Blue
line13, = ax.plot([], [], [], 'o-', lw=2, color='#ffbb78')  # Light Orange
line14, = ax.plot([], [], [], 'o-', lw=2, color='#98df8a')  # Light Green
line15, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Red
line16, = ax.plot([], [], [], 'o-', lw=2, color='#c5b0d5')  # Light Purple
line17, = ax.plot([], [], [], 'o-', lw=2, color='#c49c94')  # Light Brown
line18, = ax.plot([], [], [], 'o-', lw=2, color='#f7b6d2')  # Light Pink
line19, = ax.plot([], [], [], 'o-', lw=2, color='#c7c7c7')  # Light Gray
line20, = ax.plot([], [], [], 'o-', lw=2, color='#dbdb8d')  # Light Yellow
line21, = ax.plot([], [], [], 'o-', lw=2, color='#9edae5')  # Light Teal/Cyan

node1, = ax.plot([], [], [], 'ro', lw=2)    # Red
node2, = ax.plot([], [], [], 'yo', lw=2)    # Yellow
node3, = ax.plot([], [], [], 'go', lw=2)    # Green
node4, = ax.plot([], [], [], 'co', lw=2)    # Cyan
node5, = ax.plot([], [], [], 'bo', lw=2)    # Blue
node6, = ax.plot([], [], [], 'ko', lw=2)    # Black
node7, = ax.plot([], [], [], 'mo', lw=2)    # Magenta
node8, = ax.plot([], [], [], 'wo', lw=2)    # White
node9, = ax.plot([], [], [], 'ro', lw=2)    # Red

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 3)

plt.title("Solar Array Animation")

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
node7_x = []
node7_y = []
node7_z = []
node8_x = []
node8_y = []
node8_z = []
node9_x = []
node9_y = []
node9_z = []

# Create a second picture that plots the x, y and z coordinates of each node in time
# fig2, ax2 = plt.subplots(3, 2)

# Initialize the octahedron dynamics object (used to update the dynamics later)
octahedron = SolarDynamics()
# Initialize the rigidity matrix object, used to initialize the node positions and edges
RM = SolarRigidityMatrix()

# Initialize the disturbance input
disturbance = signalGenerator(amplitude = 100.0, frequency = 0.25, y_offset = 0.0)

# Initializtion function
def init():
    x = RM.x
    line1.set_data([x[0, 0], x[1, 0]], [x[0, 1], x[1, 1]]) # 1-2
    line1.set_3d_properties([x[0, 2], x[1, 2]])
    line2.set_data([x[1, 1], x[2, 1]], [x[1, 0], x[2, 0]]) # 3-2
    line2.set_3d_properties([x[2, 2], x[1, 2]])
    line3.set_data([x[2, 1], x[0, 1]], [x[2, 0], x[0, 0]]) # 3-1
    line3.set_3d_properties([x[0, 2], x[2, 2]])
    line4.set_data([x[4, 0], x[3, 0]], [x[4, 1], x[3, 1]]) # 5-4
    line4.set_3d_properties([x[4, 2], x[3, 2]])
    line5.set_data([x[3, 0], x[2, 0]], [x[3, 1], x[2, 1]]) # 4-3
    line5.set_3d_properties([x[3, 2], x[2, 2]])
    line6.set_data([x[2, 0], x[4, 0]], [x[2, 1], x[4, 1]]) # 3-5
    line6.set_3d_properties([x[2, 2], x[4, 2]])
    line7.set_data([x[4, 0], x[1, 0]], [x[4, 1], x[1, 1]]) # 5-2
    line7.set_3d_properties([x[4, 2], x[1, 2]])
    line8.set_data([x[1, 0], x[5, 0]], [x[1, 1], x[5, 1]]) # 2-6
    line8.set_3d_properties([x[1, 2], x[5, 2]])
    line9.set_data([x[5, 0], x[4, 0]], [x[5, 1], x[4, 1]]) # 6-5
    line9.set_3d_properties([x[5, 2], x[4, 2]])
    line10.set_data([x[0, 0], x[3, 0]], [x[0, 1], x[3, 1]]) # 1-4
    line10.set_3d_properties([x[0, 2], x[3, 2]])
    line11.set_data([x[3, 0], x[5, 0]], [x[3, 1], x[5, 1]]) # 4-6
    line11.set_3d_properties([x[3, 2], x[5, 2]])
    line12.set_data([x[5, 0], x[0, 0]], [x[5, 1], x[0, 1]]) # 6-1
    line12.set_3d_properties([x[5, 2], x[0, 2]])
    line13.set_data([x[6, 0], x[5, 0]], [x[6, 1], x[5, 1]]) # 8-7
    line13.set_3d_properties([x[6, 2], x[5, 2]])
    line14.set_data([x[6, 0], x[5, 0]], [x[6, 1], x[5, 1]]) # 7-6
    line14.set_3d_properties([x[6, 2], x[5, 2]])
    line15.set_data([x[5, 0], x[7, 0]], [x[5, 1], x[7, 1]]) # 6-8
    line15.set_3d_properties([x[5, 2], x[7, 2]])
    line16.set_data([x[7, 0], x[3, 0]], [x[7, 1], x[3, 1]]) # 8-4
    line16.set_3d_properties([x[7, 2], x[3, 2]])
    line17.set_data([x[3, 0], x[8, 0]], [x[3, 1], x[8, 1]]) # 4-9
    line17.set_3d_properties([x[3, 2], x[8, 2]])
    line18.set_data([x[8, 0], x[7, 0]], [x[8, 1], x[7, 1]]) # 9-8
    line18.set_3d_properties([x[8, 2], x[7, 2]])
    line19.set_data([x[4, 0], x[6, 0]], [x[4, 1], x[6, 1]]) # 5-7
    line19.set_3d_properties([x[4, 2], x[6, 2]])
    line20.set_data([x[6, 0], x[8, 0]], [x[6, 1], x[8, 1]]) # 7-9
    line20.set_3d_properties([x[6, 2], x[8, 2]])
    line21.set_data([x[8, 0], x[4, 0]], [x[8, 1], x[4, 1]]) # 9-5
    line21.set_3d_properties([x[8, 2], x[4, 2]])
    
    node1.set_data([x[0, 0]], [x[0, 1]])
    node1.set_3d_properties([x[0, 2]])
    node2.set_data([x[1, 0]], [x[1, 1]])
    node2.set_3d_properties(x[1, 2])
    node3.set_data([x[2, 0]], [x[2, 1]])
    node3.set_3d_properties(x[2, 2])
    node4.set_data([x[3, 0]], [x[3, 1]])
    node4.set_3d_properties(x[3, 2])
    node5.set_data([x[4, 0]], [x[4, 1]])
    node5.set_3d_properties(x[4, 2])
    node6.set_data([x[5, 0]], [x[5, 1]])
    node6.set_3d_properties(x[5, 2])
    node7.set_data([x[6, 0]], [x[6, 1]])
    node7.set_3d_properties(x[6, 2])
    node8.set_data([x[7, 0]], [x[7, 1]])
    node8.set_3d_properties(x[7, 2])
    node9.set_data([x[8, 0]], [x[8, 1]])
    node9.set_3d_properties(x[8, 2])

    global node_texts
    node_texts = [
        ax.text(float(RM.x[0,0]), float(RM.x[0,1]), float(RM.x[0,2]), '1', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[1,0]), float(RM.x[1,1]), float(RM.x[1,2]), '2', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[2,0]), float(RM.x[2,1]), float(RM.x[2,2]), '3', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[3,0]), float(RM.x[3,1]), float(RM.x[3,2]), '4', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[4,0]), float(RM.x[4,1]), float(RM.x[4,2]), '5', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[5,0]), float(RM.x[5,1]), float(RM.x[5,2]), '6', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[6,0]), float(RM.x[6,1]), float(RM.x[6,2]), '7', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[7,0]), float(RM.x[7,1]), float(RM.x[7,2]), '8', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[8,0]), float(RM.x[8,1]), float(RM.x[8,2]), '9', color='black', fontsize=12, ha='left', va='bottom')
    ]

    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, node1, node2, node3, node4, node5, node6, *node_texts

# Update function for animation
def update(frame):

    tau = np.zeros((27))
    # Select the type of input disturbance here. Options include square, step, sin (sine), random, and sawtooth.
    # parameters defining the input signal are defined on line ~76-ish in this file
    tau[18:27] = disturbance.square(frame*P.Ts)

    octahedron.update(tau)
    x, y, z = octahedron.h()

    line1.set_data([x[0,0], x[1,0]], [y[0,0], y[1,0]]) #1-2
    line1.set_3d_properties([z[0,0], z[1,0]])
    line2.set_data([x[2,0], x[1,0]], [y[2,0], y[1,0]]) #2-3
    line2.set_3d_properties([z[2,0], z[1,0]])
    line3.set_data([x[0,0], x[2,0]], [y[0,0], y[2,0]]) #3-1
    line3.set_3d_properties([z[0,0], z[2,0]])
    line4.set_data([x[4,0], x[3,0]], [y[4,0], y[3,0]]) #5-4
    line4.set_3d_properties([z[4,0], z[3,0]])
    line5.set_data([x[3,0], x[2,0]], [y[3,0], y[2,0]]) #4-3
    line5.set_3d_properties([z[3,0], z[2,0]])
    line6.set_data([x[2,0], x[4,0]], [y[2,0], y[4,0]]) #3-5
    line6.set_3d_properties([z[2,0], z[4,0]])
    line7.set_data([x[4,0], x[1,0]], [y[4,0], y[1,0]]) #5-2
    line7.set_3d_properties([z[4,0], z[1,0]])
    line8.set_data([x[1,0], x[5,0]], [y[1,0], y[5,0]]) #2-6
    line8.set_3d_properties([z[1,0], z[5,0]])
    line9.set_data([x[5,0], x[4,0]], [y[5,0], y[4,0]]) #6-5
    line9.set_3d_properties([z[5,0], z[4,0]])
    line10.set_data([x[0,0], x[3,0]], [y[0,0], y[3,0]]) #1-4
    line10.set_3d_properties([z[0,0], z[3,0]])
    line11.set_data([x[3,0], x[5,0]], [y[3,0], y[5,0]]) #4-6
    line11.set_3d_properties([z[3,0], z[5,0]])
    line12.set_data([x[5,0], x[0,0]], [y[5,0], y[0,0]]) #6-1
    line12.set_3d_properties([z[5,0], z[0,0]])
    line13.set_data([x[7,0], x[6,0]], [y[7,0], y[6,0]]) #8-7
    line13.set_3d_properties([z[7,0], z[6,0]])
    line14.set_data([x[6,0], x[5,0]], [y[6,0], y[5,0]]) #7-6
    line14.set_3d_properties([z[6,0], z[5,0]])
    line15.set_data([x[5,0], x[7,0]], [y[5,0], y[7,0]]) #6-8
    line15.set_3d_properties([z[5,0], z[7,0]])
    line16.set_data([x[7,0], x[3,0]], [y[7,0], y[3,0]]) #8-4
    line16.set_3d_properties([z[7,0], z[3,0]])
    line17.set_data([x[3,0], x[8,0]], [y[3,0], y[8,0]]) #4-9
    line17.set_3d_properties([z[3,0], z[8,0]])
    line18.set_data([x[8,0], x[7,0]], [y[8,0], y[7,0]]) #9-8
    line18.set_3d_properties([z[8,0], z[7,0]])
    line19.set_data([x[4,0], x[6,0]], [y[4,0], y[6,0]]) #5-7
    line19.set_3d_properties([z[4,0], z[6,0]])
    line20.set_data([x[6,0], x[8,0]], [y[6,0], y[8,0]]) #7-9
    line20.set_3d_properties([z[6,0], z[8,0]])
    line21.set_data([x[8,0], x[4,0]], [y[8,0], y[4,0]]) #9-5
    line21.set_3d_properties([z[8,0], z[4,0]])

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
    node7.set_data(x[6], y[6])
    node7.set_3d_properties(z[6])
    node8.set_data(x[7], y[7])
    node8.set_3d_properties(z[7])
    node9.set_data(x[8], y[8])
    node9.set_3d_properties(z[8])

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
    node7_x.append(x[6])
    node7_y.append(y[6])
    node7_z.append(z[6])
    node8_x.append(x[7])
    node8_y.append(y[7])
    node8_z.append(z[7])
    node9_x.append(x[8])
    node9_y.append(y[8])
    node9_z.append(z[8])

    for idx, text in enumerate(node_texts):
        text.set_position((float(x[idx]), float(y[idx])))
        text.set_3d_properties(float(z[idx]), zdir='x')

    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15, line16, line17, line18, line19, line20, line21, node1, node2, node3, node4, node5, node6, node7, node8, node9, *node_texts

def save_animation(dynamic_animation):
    f = r"C:/Users/stowel22/Desktop/animation.mp4"
    FFMpegWriter = animation.writers['ffmpeg']
    writer = FFMpegWriter(fps=30, metadata=dict(artist='stowel22'), bitrate=1800)
    dynamic_animation.save(f, writer=writer)

dynamic_animation = FuncAnimation(fig, update, frames = P.n_steps, init_func=init, blit=True, interval=1000*P.Ts)

# print("")
# userInput = input("Save animation? (y/n): ")
# if userInput == 'y':
#     save_animation(dynamic_animation)
plt.show()

# Create a second figure that plots the x, y and z coordinates of each node in time
fig2, axs = plt.subplots(9, 3, figsize = (10, 12))
axs[0, 0].set_title("X")
axs[0, 1].set_title("Y")
axs[0, 2].set_title("Z")
axs[0, 0].set_ylabel("Node 1")
axs[1, 0].set_ylabel("Node 2")
axs[2, 0].set_ylabel("Node 3")
axs[3, 0].set_ylabel("Node 4")
axs[4, 0].set_ylabel("Node 5")
axs[5, 0].set_ylabel("Node 6")
axs[6, 0].set_ylabel("Node 7")
axs[7, 0].set_ylabel("Node 8")
axs[8, 0].set_ylabel("Node 9")


# for i in range(6):
#     for j in range(3):
#         axs[i, j].set_xlim(0, P.n_steps*P.Ts)
#         axs[i, j].set_ylim(-0.5, 2)

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
axs[6, 0].plot(time, node7_x)
axs[6, 1].plot(time, node7_y)
axs[6, 2].plot(time, node7_z)
axs[7, 0].plot(time, node8_x)
axs[7, 1].plot(time, node8_y)
axs[7, 2].plot(time, node8_z)
axs[8, 0].plot(time, node9_x)
axs[8, 1].plot(time, node9_y)
axs[8, 2].plot(time, node9_z)

plt.show()