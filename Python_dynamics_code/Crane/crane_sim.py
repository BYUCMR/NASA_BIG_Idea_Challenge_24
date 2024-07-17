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
line22, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Salmon
line23, = ax.plot([], [], [], 'o-', lw=2, color='#aec7e8')  # Light Blue
line24, = ax.plot([], [], [], 'o-', lw=2, color='#ffbb78')  # Light Orange
line25, = ax.plot([], [], [], 'o-', lw=2, color='#98df8a')  # Light Green
line26, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Red
line27, = ax.plot([], [], [], 'o-', lw=2, color='#c5b0d5')  # Light Purple
line28, = ax.plot([], [], [], 'o-', lw=2, color='#c49c94')  # Light Brown
line29, = ax.plot([], [], [], 'o-', lw=2, color='#f7b6d2')  # Light Pink
line30, = ax.plot([], [], [], 'o-', lw=2, color='#c7c7c7')  # Light Gray
line31, = ax.plot([], [], [], 'o-', lw=2, color='#dbdb8d')  # Light Yellow
line32, = ax.plot([], [], [], 'o-', lw=2, color='#9edae5')  # Light Teal/Cyan
line33, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Salmon
line34, = ax.plot([], [], [], 'o-', lw=2, color='#aec7e8')  # Light Blue
line35, = ax.plot([], [], [], 'o-', lw=2, color='#ffbb78')  # Light Orange
line36, = ax.plot([], [], [], 'o-', lw=2, color='#98df8a')  # Light Green
line37, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Red
line38, = ax.plot([], [], [], 'o-', lw=2, color='#c5b0d5')  # Light Purple
line39, = ax.plot([], [], [], 'o-', lw=2, color='#c49c94')  # Light Brown
line40, = ax.plot([], [], [], 'o-', lw=2, color='#f7b6d2')  # Light Pink
line41, = ax.plot([], [], [], 'o-', lw=2, color='#c7c7c7')  # Light Gray
line42, = ax.plot([], [], [], 'o-', lw=2, color='#dbdb8d')  # Light Yellow
line43, = ax.plot([], [], [], 'o-', lw=2, color='#9edae5')  # Light Teal/Cyan
line44, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Salmon
line45, = ax.plot([], [], [], 'o-', lw=2, color='#aec7e8')  # Light Blue
line46, = ax.plot([], [], [], 'o-', lw=2, color='#ffbb78')  # Light Orange
line47, = ax.plot([], [], [], 'o-', lw=2, color='#98df8a')  # Light Green
line48, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Red
line49, = ax.plot([], [], [], 'o-', lw=2, color='#c5b0d5')  # Light Purple
line50, = ax.plot([], [], [], 'o-', lw=2, color='#c49c94')  # Light Brown
line51, = ax.plot([], [], [], 'o-', lw=2, color='#f7b6d2')  # Light Pink
line52, = ax.plot([], [], [], 'o-', lw=2, color='#c7c7c7')  # Light Gray
line53, = ax.plot([], [], [], 'o-', lw=2, color='#dbdb8d')  # Light Yellow
line54, = ax.plot([], [], [], 'o-', lw=2, color='#9edae5')  # Light Teal/Cyan
line55, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Salmon
line56, = ax.plot([], [], [], 'o-', lw=2, color='#aec7e8')  # Light Blue
line57, = ax.plot([], [], [], 'o-', lw=2, color='#ffbb78')  # Light Orange
line58, = ax.plot([], [], [], 'o-', lw=2, color='#98df8a')  # Light Green
line59, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Red
line60, = ax.plot([], [], [], 'o-', lw=2, color='#c5b0d5')  # Light Purple
line61, = ax.plot([], [], [], 'o-', lw=2, color='#c49c94')  # Light Brown
line62, = ax.plot([], [], [], 'o-', lw=2, color='#f7b6d2')  # Light Pink
line63, = ax.plot([], [], [], 'o-', lw=2, color='#c7c7c7')  # Light Gray
line64, = ax.plot([], [], [], 'o-', lw=2, color='#dbdb8d')  # Light Yellow
line65, = ax.plot([], [], [], 'o-', lw=2, color='#9edae5')  # Light Teal/Cyan
line66, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Salmon
line67, = ax.plot([], [], [], 'o-', lw=2, color='#aec7e8')  # Light Blue
line68, = ax.plot([], [], [], 'o-', lw=2, color='#ffbb78')  # Light Orange
line69, = ax.plot([], [], [], 'o-', lw=2, color='#98df8a')  # Light Green
line70, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Red
line71, = ax.plot([], [], [], 'o-', lw=2, color='#c5b0d5')  # Light Purple
line72, = ax.plot([], [], [], 'o-', lw=2, color='#c49c94')  # Light Brown
line73, = ax.plot([], [], [], 'o-', lw=2, color='#f7b6d2')  # Light Pink
line74, = ax.plot([], [], [], 'o-', lw=2, color='#c7c7c7')  # Light Gray
line75, = ax.plot([], [], [], 'o-', lw=2, color='#dbdb8d')  # Light Yellow
line76, = ax.plot([], [], [], 'o-', lw=2, color='#9edae5')  # Light Teal/Cyan
line77, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Salmon
line78, = ax.plot([], [], [], 'o-', lw=2, color='#aec7e8')  # Light Blue
line79, = ax.plot([], [], [], 'o-', lw=2, color='#ffbb78')  # Light Orange
line80, = ax.plot([], [], [], 'o-', lw=2, color='#98df8a')  # Light Green
line81, = ax.plot([], [], [], 'o-', lw=2, color='#ff9896')  # Light Red
line82, = ax.plot([], [], [], 'o-', lw=2, color='#c5b0d5')  # Light Purple
line83, = ax.plot([], [], [], 'o-', lw=2, color='#c49c94')  # Light Brown
line84, = ax.plot([], [], [], 'o-', lw=2, color='#f7b6d2')  # Light Pink


node1, = ax.plot([], [], [], 'ro', lw=2)    # Red
node2, = ax.plot([], [], [], 'yo', lw=2)    # Yellow
node3, = ax.plot([], [], [], 'go', lw=2)    # Green
node4, = ax.plot([], [], [], 'co', lw=2)    # Cyan
node5, = ax.plot([], [], [], 'bo', lw=2)    # Blue
node6, = ax.plot([], [], [], 'ko', lw=2)    # Black
node7, = ax.plot([], [], [], 'mo', lw=2)    # Magenta
node8, = ax.plot([], [], [], 'wo', lw=2)    # White
node9, = ax.plot([], [], [], 'ro', lw=2)    # Red
node10, = ax.plot([], [], [], 'yo', lw=2)   # Yellow
node11, = ax.plot([], [], [], 'go', lw=2)   # Green
node12, = ax.plot([], [], [], 'co', lw=2)   # Cyan
node13, = ax.plot([], [], [], 'bo', lw=2)   # Blue
node14, = ax.plot([], [], [], 'ko', lw=2)   # Black
node15, = ax.plot([], [], [], 'mo', lw=2)   # Magenta
node16, = ax.plot([], [], [], 'wo', lw=2)   # White
node17, = ax.plot([], [], [], 'ro', lw=2)   # Red
node18, = ax.plot([], [], [], 'yo', lw=2)   # Yellow
node19, = ax.plot([], [], [], 'go', lw=2)   # Green
node20, = ax.plot([], [], [], 'co', lw=2)   # Cyan
node21, = ax.plot([], [], [], 'bo', lw=2)   # Blue
node22, = ax.plot([], [], [], 'ko', lw=2)   # Black
node23, = ax.plot([], [], [], 'mo', lw=2)   # Magenta
node24, = ax.plot([], [], [], 'wo', lw=2)   # White
node25, = ax.plot([], [], [], 'ro', lw=2)   # Red
node26, = ax.plot([], [], [], 'yo', lw=2)   # Yellow
node27, = ax.plot([], [], [], 'go', lw=2)   # Green
node28, = ax.plot([], [], [], 'co', lw=2)   # Cyan
node29, = ax.plot([], [], [], 'bo', lw=2)   # Blue
node30, = ax.plot([], [], [], 'ko', lw=2)   # Black

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-3, 4)

plt.title("Crane Animation")

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
node10_x = []
node10_y = []
node10_z = []
node11_x = []
node11_y = []
node11_z = []
node12_x = []
node12_y = []
node12_z = []
node13_x = []
node13_y = []
node13_z = []
node14_x = []
node14_y = []
node14_z = []
node15_x = []
node15_y = []
node15_z = []
node16_x = []
node16_y = []
node16_z = []
node17_x = []
node17_y = []
node17_z = []
node18_x = []
node18_y = []
node18_z = []
node19_x = []
node19_y = []
node19_z = []
node20_x = []
node20_y = []
node20_z = []
node21_x = []
node21_y = []
node21_z = []
node22_x = []
node22_y = []
node22_z = []
node23_x = []
node23_y = []
node23_z = []
node24_x = []
node24_y = []
node24_z = []
node25_x = []
node25_y = []
node25_z = []
node26_x = []
node26_y = []
node26_z = []
node27_x = []
node27_y = []
node27_z = []
node28_x = []
node28_y = []
node28_z = []
node29_x = []
node29_y = []
node29_z = []
node30_x = []
node30_y = []
node30_z = []

# Create a second picture that plots the x, y and z coordinates of each node in time
# fig2, ax2 = plt.subplots(3, 2)

# Initialize the octahedron dynamics object (used to update the dynamics later)
crane = CraneDynamics()
# Initialize the rigidity matrix object, used to initialize the node positions and edges
RM = CraneRigidityMatrix()

# Initialize the disturbance input
disturbance = signalGenerator(amplitude = 100.0, frequency = 0.25, y_offset = 0.0)

# Initializtion function
def init():
    x = RM.x
    line1.set_data([x[0, 0], x[1, 0]], [x[0, 1], x[1, 1]]) # 1-2
    line1.set_3d_properties([x[0, 2], x[1, 2]])
    line2.set_data([x[2, 1], x[1, 1]], [x[2, 0], x[1, 0]]) # 3-2
    line2.set_3d_properties([x[2, 2], x[1, 2]])
    line3.set_data([x[2, 1], x[0, 1]], [x[2, 0], x[0, 0]]) # 3-1
    line3.set_3d_properties([x[2, 2], x[0, 2]])
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
    line14.set_data([x[6, 0], x[2, 0]], [x[6, 1], x[2, 1]]) # 7-3
    line14.set_3d_properties([x[6, 2], x[2, 2]])
    line15.set_data([x[2, 0], x[7, 0]], [x[2, 1], x[7, 1]]) # 3-8
    line15.set_3d_properties([x[2, 2], x[7, 2]])
    line16.set_data([x[7, 0], x[4, 0]], [x[7, 1], x[4, 1]]) # 8-5
    line16.set_3d_properties([x[7, 2], x[4, 2]])
    line17.set_data([x[4, 0], x[8, 0]], [x[4, 1], x[8, 1]]) # 5-9
    line17.set_3d_properties([x[4, 2], x[8, 2]])
    line18.set_data([x[8, 0], x[7, 0]], [x[8, 1], x[7, 1]]) # 9-8
    line18.set_3d_properties([x[8, 2], x[7, 2]])
    line19.set_data([x[3, 0], x[6, 0]], [x[3, 1], x[6, 1]]) # 4-7
    line19.set_3d_properties([x[3, 2], x[6, 2]])
    line20.set_data([x[6, 0], x[8, 0]], [x[6, 1], x[8, 1]]) # 7-9
    line20.set_3d_properties([x[6, 2], x[8, 2]])
    line21.set_data([x[8, 0], x[3, 0]], [x[8, 1], x[3, 1]]) # 9-4
    line21.set_3d_properties([x[8, 2], x[3, 2]])
    line22.set_data([x[10, 0], x[9, 0]], [x[10, 1], x[9, 1]]) # 11-10
    line22.set_3d_properties([x[10, 2], x[9, 2]])
    line23.set_data([x[9, 0], x[1, 0]], [x[9, 1], x[1, 1]]) # 10-2
    line23.set_3d_properties([x[9, 2], x[1, 2]])
    line24.set_data([x[1, 0], x[10, 0]], [x[1, 1], x[10, 1]]) # 2-11
    line24.set_3d_properties([x[1, 2], x[10, 2]])
    line25.set_data([x[10, 0], x[2, 0]], [x[10, 1], x[2, 1]]) # 11-3
    line25.set_3d_properties([x[10, 2], x[2, 2]])
    line26.set_data([x[2, 0], x[11, 0]], [x[2, 1], x[11, 1]]) # 3-12
    line26.set_3d_properties([x[2, 2], x[11, 2]])
    line27.set_data([x[11, 0], x[10, 0]], [x[11, 1], x[10, 1]]) # 12-11
    line27.set_3d_properties([x[11, 2], x[10, 2]])
    line28.set_data([x[0, 0], x[9, 0]], [x[0, 1], x[9, 1]]) # 1-10
    line28.set_3d_properties([x[0, 2], x[9, 2]])
    line29.set_data([x[9, 0], x[11, 0]], [x[9, 1], x[11, 1]]) # 10-12
    line29.set_3d_properties([x[9, 2], x[11, 2]])
    line30.set_data([x[11, 0], x[0, 0]], [x[11, 1], x[0, 1]]) # 12-1
    line30.set_3d_properties([x[11, 2], x[0, 2]])
    line31.set_data([x[13, 0], x[12, 0]], [x[13, 1], x[12, 1]]) # 14-13
    line31.set_3d_properties([x[13, 2], x[12, 2]])
    line32.set_data([x[12, 0], x[1, 0]], [x[12, 1], x[1, 1]]) # 13-2
    line32.set_3d_properties([x[12, 2], x[1, 2]])
    line33.set_data([x[1, 0], x[13, 0]], [x[1, 1], x[13, 1]]) # 2-14
    line33.set_3d_properties([x[1, 2], x[13, 2]])
    line34.set_data([x[13, 0], x[4, 0]], [x[13, 1], x[4, 1]]) # 14-5
    line34.set_3d_properties([x[13, 2], x[4, 2]])
    line35.set_data([x[4, 0], x[14, 0]], [x[4, 1], x[14, 1]]) # 5-15
    line35.set_3d_properties([x[4, 2], x[14, 2]])
    line36.set_data([x[14, 0], x[13, 0]], [x[14, 1], x[13, 1]]) # 15-14
    line36.set_3d_properties([x[14, 2], x[13, 2]])
    line37.set_data([x[2, 0], x[12, 0]], [x[2, 1], x[12, 1]]) # 3-13
    line37.set_3d_properties([x[2, 2], x[12, 2]])
    line38.set_data([x[12, 0], x[14, 0]], [x[12, 1], x[14, 1]]) # 13-15
    line38.set_3d_properties([x[12, 2], x[14, 2]])
    line39.set_data([x[14, 0], x[2, 0]], [x[14, 1], x[2, 1]]) # 15-3
    line39.set_3d_properties([x[14, 2], x[2, 2]])
    line40.set_data([x[16, 0], x[15, 0]], [x[16, 1], x[15, 1]]) # 17-16
    line40.set_3d_properties([x[16, 2], x[15, 2]])
    line41.set_data([x[15, 0], x[2, 0]], [x[15, 1], x[2, 1]]) # 16-3
    line41.set_3d_properties([x[15, 2], x[2, 2]])
    line42.set_data([x[2, 0], x[16, 0]], [x[2, 1], x[16, 1]]) # 3-17
    line42.set_3d_properties([x[2, 2], x[16, 2]])
    line43.set_data([x[16, 0], x[3, 0]], [x[16, 1], x[3, 1]]) # 17-4
    line43.set_3d_properties([x[16, 2], x[3, 2]])
    line44.set_data([x[3, 0], x[17, 0]], [x[3, 1], x[17, 1]]) # 4-18
    line44.set_3d_properties([x[3, 2], x[17, 2]])
    line45.set_data([x[17, 0], x[16, 0]], [x[17, 1], x[16, 1]]) # 18-17
    line45.set_3d_properties([x[17, 2], x[16, 2]])
    line46.set_data([x[0, 0], x[15, 0]], [x[0, 1], x[15, 1]]) # 1-16
    line46.set_3d_properties([x[0, 2], x[15, 2]])
    line47.set_data([x[15, 0], x[17, 0]], [x[15, 1], x[17, 1]]) # 16-18
    line47.set_3d_properties([x[15, 2], x[17, 2]])
    line48.set_data([x[17, 0], x[0, 0]], [x[17, 1], x[0, 1]]) # 18-1
    line48.set_3d_properties([x[17, 2], x[0, 2]])
    line49.set_data([x[19, 0], x[18, 0]], [x[19, 1], x[18, 1]]) # 20-19
    line49.set_3d_properties([x[19, 2], x[18, 2]])
    line50.set_data([x[18, 0], x[10, 0]], [x[18, 1], x[10, 1]]) # 19-11
    line50.set_3d_properties([x[18, 2], x[10, 2]])
    line51.set_data([x[10, 0], x[19, 0]], [x[10, 1], x[19, 1]]) # 11-20
    line51.set_3d_properties([x[10, 2], x[19, 2]])
    line52.set_data([x[19, 0], x[11, 0]], [x[19, 1], x[11, 1]]) # 20-12
    line52.set_3d_properties([x[19, 2], x[11, 2]])
    line53.set_data([x[11, 0], x[20, 0]], [x[11, 1], x[20, 1]]) # 12-21
    line53.set_3d_properties([x[11, 2], x[20, 2]])
    line54.set_data([x[20, 0], x[19, 0]], [x[20, 1], x[19, 1]]) # 21-20
    line54.set_3d_properties([x[20, 2], x[19, 2]])
    line55.set_data([x[9, 0], x[18, 0]], [x[9, 1], x[18, 1]]) # 10-19
    line55.set_3d_properties([x[9, 2], x[18, 2]])
    line56.set_data([x[18, 0], x[20, 0]], [x[18, 1], x[20, 1]]) # 19-21
    line56.set_3d_properties([x[18, 2], x[20, 2]])
    line57.set_data([x[20, 0], x[9, 0]], [x[20, 1], x[9, 1]]) # 21-10
    line57.set_3d_properties([x[20, 2], x[9, 2]])
    line58.set_data([x[22, 0], x[21, 0]], [x[22, 1], x[21, 1]]) # 23-22
    line58.set_3d_properties([x[22, 2], x[21, 2]])
    line59.set_data([x[21, 0], x[13, 0]], [x[21, 1], x[13, 1]]) # 22-14
    line59.set_3d_properties([x[21, 2], x[13, 2]])
    line60.set_data([x[13, 0], x[22, 0]], [x[13, 1], x[22, 1]]) # 14-23
    line60.set_3d_properties([x[13, 2], x[22, 2]])
    line61.set_data([x[22, 0], x[14, 0]], [x[22, 1], x[14, 1]]) # 23-15
    line61.set_3d_properties([x[22, 2], x[14, 2]])
    line62.set_data([x[14, 0], x[23, 0]], [x[14, 1], x[23, 1]]) # 15-24
    line62.set_3d_properties([x[14, 2], x[23, 2]])
    line63.set_data([x[23, 0], x[22, 0]], [x[23, 1], x[22, 1]]) # 24-23
    line63.set_3d_properties([x[23, 2], x[22, 2]])
    line64.set_data([x[12, 0], x[21, 0]], [x[12, 1], x[21, 1]]) # 13-22
    line64.set_3d_properties([x[12, 2], x[21, 2]])
    line65.set_data([x[21, 0], x[23, 0]], [x[21, 1], x[23, 1]]) # 22-24
    line65.set_3d_properties([x[21, 2], x[23, 2]])
    line66.set_data([x[23, 0], x[12, 0]], [x[23, 1], x[12, 1]]) # 24-13
    line66.set_3d_properties([x[23, 2], x[12, 2]])
    line67.set_data([x[25, 0], x[24, 0]], [x[25, 1], x[24, 1]]) # 26-25
    line67.set_3d_properties([x[25, 2], x[24, 2]])
    line68.set_data([x[24, 0], x[7, 0]], [x[24, 1], x[7, 1]]) # 25-8
    line68.set_3d_properties([x[24, 2], x[7, 2]])
    line69.set_data([x[7, 0], x[25, 0]], [x[7, 1], x[25, 1]]) # 8-26
    line69.set_3d_properties([x[7, 2], x[25, 2]])
    line70.set_data([x[25, 0], x[8, 0]], [x[25, 1], x[8, 1]]) # 26-9
    line70.set_3d_properties([x[25, 2], x[8, 2]])
    line71.set_data([x[8, 0], x[26, 0]], [x[8, 1], x[26, 1]]) # 9-27
    line71.set_3d_properties([x[8, 2], x[26, 2]])
    line72.set_data([x[26, 0], x[25, 0]], [x[26, 1], x[25, 1]]) # 27-26
    line72.set_3d_properties([x[26, 2], x[25, 2]])
    line73.set_data([x[6, 0], x[24, 0]], [x[6, 1], x[24, 1]]) # 7-25
    line73.set_3d_properties([x[6, 2], x[24, 2]])
    line74.set_data([x[24, 0], x[26, 0]], [x[24, 1], x[26, 1]]) # 25-27
    line74.set_3d_properties([x[24, 2], x[26, 2]])
    line75.set_data([x[26, 0], x[6, 0]], [x[26, 1], x[6, 1]]) # 27-7
    line75.set_3d_properties([x[26, 2], x[6, 2]])
    line76.set_data([x[28, 0], x[27, 0]], [x[28, 1], x[27, 1]]) # 29-28
    line76.set_3d_properties([x[28, 2], x[27, 2]])
    line77.set_data([x[27, 0], x[16, 0]], [x[27, 1], x[16, 1]]) # 28-17
    line77.set_3d_properties([x[27, 2], x[16, 2]])
    line78.set_data([x[16, 0], x[28, 0]], [x[16, 1], x[28, 1]]) # 17-29
    line78.set_3d_properties([x[16, 2], x[28, 2]])
    line79.set_data([x[28, 0], x[17, 0]], [x[28, 1], x[17, 1]]) # 29-18
    line79.set_3d_properties([x[28, 2], x[17, 2]])
    line80.set_data([x[17, 0], x[29, 0]], [x[17, 1], x[29, 1]]) # 18-30
    line80.set_3d_properties([x[17, 2], x[29, 2]])
    line81.set_data([x[29, 0], x[28, 0]], [x[29, 1], x[28, 1]]) # 30-29
    line81.set_3d_properties([x[29, 2], x[28, 2]])
    line82.set_data([x[15, 0], x[27, 0]], [x[15, 1], x[27, 1]]) # 16-28
    line82.set_3d_properties([x[15, 2], x[27, 2]])
    line83.set_data([x[27, 0], x[29, 0]], [x[27, 1], x[29, 1]]) # 28-30
    line83.set_3d_properties([x[27, 2], x[29, 2]])
    line84.set_data([x[29, 0], x[15, 0]], [x[29, 1], x[15, 1]]) # 30-16
    line84.set_3d_properties([x[29, 2], x[15, 2]])
                             
    
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
    node10.set_data([x[9, 0]], [x[9, 1]])
    node10.set_3d_properties(x[9, 2])
    node11.set_data([x[10, 0]], [x[10, 1]])
    node11.set_3d_properties(x[10, 2])
    node12.set_data([x[11, 0]], [x[11, 1]])
    node12.set_3d_properties(x[11, 2])
    node13.set_data([x[12, 0]], [x[12, 1]])
    node13.set_3d_properties(x[12, 2])
    node14.set_data([x[13, 0]], [x[13, 1]])
    node14.set_3d_properties(x[13, 2])
    node15.set_data([x[14, 0]], [x[14, 1]])
    node15.set_3d_properties(x[14, 2])
    node16.set_data([x[15, 0]], [x[15, 1]])
    node16.set_3d_properties(x[15, 2])
    node17.set_data([x[16, 0]], [x[16, 1]])
    node17.set_3d_properties(x[16, 2])
    node18.set_data([x[17, 0]], [x[17, 1]])
    node18.set_3d_properties(x[17, 2])
    node19.set_data([x[18, 0]], [x[18, 1]])
    node19.set_3d_properties(x[18, 2])
    node20.set_data([x[19, 0]], [x[19, 1]])
    node20.set_3d_properties(x[19, 2])
    node21.set_data([x[20, 0]], [x[20, 1]])
    node21.set_3d_properties(x[20, 2])
    node22.set_data([x[21, 0]], [x[21, 1]])
    node22.set_3d_properties(x[21, 2])
    node23.set_data([x[22, 0]], [x[22, 1]])
    node23.set_3d_properties(x[22, 2])
    node24.set_data([x[23, 0]], [x[23, 1]])
    node24.set_3d_properties(x[23, 2])
    node25.set_data([x[24, 0]], [x[24, 1]])
    node25.set_3d_properties(x[24, 2])
    node26.set_data([x[25, 0]], [x[25, 1]])
    node26.set_3d_properties(x[25, 2])
    node27.set_data([x[26, 0]], [x[26, 1]])
    node27.set_3d_properties(x[26, 2])
    node28.set_data([x[27, 0]], [x[27, 1]])
    node28.set_3d_properties(x[27, 2])
    node29.set_data([x[28, 0]], [x[28, 1]])
    node29.set_3d_properties(x[28, 2])
    node30.set_data([x[29, 0]], [x[29, 1]])
    node30.set_3d_properties(x[29, 2])

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
        ax.text(float(RM.x[8,0]), float(RM.x[8,1]), float(RM.x[8,2]), '9', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[9,0]), float(RM.x[9,1]), float(RM.x[9,2]), '10', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[10,0]), float(RM.x[10,1]), float(RM.x[10,2]), '11', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[11,0]), float(RM.x[11,1]), float(RM.x[11,2]), '12', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[12,0]), float(RM.x[12,1]), float(RM.x[12,2]), '13', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[13,0]), float(RM.x[13,1]), float(RM.x[13,2]), '14', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[14,0]), float(RM.x[14,1]), float(RM.x[14,2]), '15', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[15,0]), float(RM.x[15,1]), float(RM.x[15,2]), '16', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[16,0]), float(RM.x[16,1]), float(RM.x[16,2]), '17', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[17,0]), float(RM.x[17,1]), float(RM.x[17,2]), '18', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[18,0]), float(RM.x[18,1]), float(RM.x[18,2]), '19', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[19,0]), float(RM.x[19,1]), float(RM.x[19,2]), '20', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[20,0]), float(RM.x[20,1]), float(RM.x[20,2]), '21', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[21,0]), float(RM.x[21,1]), float(RM.x[21,2]), '22', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[22,0]), float(RM.x[22,1]), float(RM.x[22,2]), '23', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[23,0]), float(RM.x[23,1]), float(RM.x[23,2]), '24', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[24,0]), float(RM.x[24,1]), float(RM.x[24,2]), '25', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[25,0]), float(RM.x[25,1]), float(RM.x[25,2]), '26', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[26,0]), float(RM.x[26,1]), float(RM.x[26,2]), '27', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[27,0]), float(RM.x[27,1]), float(RM.x[27,2]), '28', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[28,0]), float(RM.x[28,1]), float(RM.x[28,2]), '29', color='black', fontsize=12, ha='left', va='bottom'),
        ax.text(float(RM.x[29,0]), float(RM.x[29,1]), float(RM.x[29,2]), '30', color='black', fontsize=12, ha='left', va='bottom')
    ]

    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15, line16, line17, line18, line19, line20, line21, line22, line23, line24, line25, line26, line27, line28, line29, line30, line31, line32, line33, line34, line35, line36, line37, line38, line39, line40, line41, line42, line43, line44, line45, line46, line47, line48, line49, line50, line51, line52, line53, line54, line55, line56, line57, line58, line59, line60, line61, line62, line63, line64, line65, line66, line67, line68, line69, line70, line71, line72, line73, line74, line75, line76, line77, line78, line79, line80, line81, line82, line83, line84, node1, node2, node3, node4, node5, node6, node7, node8, node9, node10, node11, node12, node13, node14, node15, node16, node17, node18, node19, node20, node21, node22, node23, node24, node25, node26, node27, node28, node29, node30, *node_texts

# Update function for animation
def update(frame):

    tau = np.zeros((90))
    # Select the type of input disturbance here. Options include square, step, sin (sine), random, and sawtooth.
    # parameters defining the input signal are defined on line ~76-ish in this file
    tau[60:90] = disturbance.square(frame*P.Ts)

    crane.update(tau)
    x, y, z = crane.h()

    line1.set_data([x[0,0], x[1,0]], [y[0,0], y[1,0]]) #1-2
    line1.set_3d_properties([z[0,0], z[1,0]])
    line2.set_data([x[1,0], x[2,0]], [y[1,0], y[2,0]]) #2-3
    line2.set_3d_properties([z[1,0], z[2,0]])
    line3.set_data([x[2,0], x[0,0]], [y[2,0], y[0,0]]) #3-1
    line3.set_3d_properties([z[2,0], z[0,0]])
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
    line14.set_data([x[6,0], x[2,0]], [y[6,0], y[2,0]]) #7-3
    line14.set_3d_properties([z[6,0], z[2,0]])
    line15.set_data([x[2,0], x[7,0]], [y[2,0], y[7,0]]) #3-8
    line15.set_3d_properties([z[2,0], z[7,0]])
    line16.set_data([x[7,0], x[4,0]], [y[7,0], y[4,0]]) #8-5
    line16.set_3d_properties([z[7,0], z[4,0]])
    line17.set_data([x[4,0], x[8,0]], [y[4,0], y[8,0]]) #5-9
    line17.set_3d_properties([z[4,0], z[8,0]])
    line18.set_data([x[8,0], x[7,0]], [y[8,0], y[7,0]]) #9-8
    line18.set_3d_properties([z[8,0], z[7,0]])
    line19.set_data([x[3,0], x[6,0]], [y[3,0], y[6,0]]) #4-7
    line19.set_3d_properties([z[3,0], z[6,0]])
    line20.set_data([x[6,0], x[8,0]], [y[6,0], y[8,0]]) #7-9
    line20.set_3d_properties([z[6,0], z[8,0]])
    line21.set_data([x[8,0], x[3,0]], [y[8,0], y[3,0]]) #9-4
    line21.set_3d_properties([z[8,0], z[3,0]])
    line22.set_data([x[10,0], x[9,0]], [y[10,0], y[9,0]]) #11-10
    line22.set_3d_properties([z[10,0], z[9,0]])
    line23.set_data([x[9,0], x[1,0]], [y[9,0], y[1,0]]) #10-2
    line23.set_3d_properties([z[9,0], z[1,0]])
    line24.set_data([x[1,0], x[10,0]], [y[1,0], y[10,0]]) #2-11
    line24.set_3d_properties([z[1,0], z[10,0]])
    line25.set_data([x[10,0], x[2,0]], [y[10,0], y[2,0]]) #11-3
    line25.set_3d_properties([z[10,0], z[2,0]])
    line26.set_data([x[2,0], x[11,0]], [y[2,0], y[11,0]]) #3-12
    line26.set_3d_properties([z[2,0], z[11,0]])
    line27.set_data([x[11,0], x[10,0]], [y[11,0], y[10,0]]) #12-11
    line27.set_3d_properties([z[11,0], z[10,0]])
    line28.set_data([x[0,0], x[9,0]], [y[0,0], y[9,0]]) #1-10
    line28.set_3d_properties([z[0,0], z[9,0]])
    line29.set_data([x[9,0], x[11,0]], [y[9,0], y[11,0]]) #10-12
    line29.set_3d_properties([z[9,0], z[11,0]])
    line30.set_data([x[11,0], x[0,0]], [y[11,0], y[0,0]]) #12-1
    line30.set_3d_properties([z[11,0], z[0,0]])
    line31.set_data([x[13,0], x[12,0]], [y[13,0], y[12,0]]) #14-13
    line31.set_3d_properties([z[13,0], z[12,0]])
    line32.set_data([x[12,0], x[1,0]], [y[12,0], y[1,0]]) #13-2
    line32.set_3d_properties([z[12,0], z[1,0]])
    line33.set_data([x[1,0], x[13,0]], [y[1,0], y[13,0]]) #2-14
    line33.set_3d_properties([z[1,0], z[13,0]])
    line34.set_data([x[13,0], x[4,0]], [y[13,0], y[4,0]]) #14-5
    line34.set_3d_properties([z[13,0], z[4,0]])
    line35.set_data([x[4,0], x[14,0]], [y[4,0], y[14,0]]) #5-15
    line35.set_3d_properties([z[4,0], z[14,0]])
    line36.set_data([x[14,0], x[13,0]], [y[14,0], y[13,0]]) #15-14
    line36.set_3d_properties([z[14,0], z[13,0]])
    line37.set_data([x[2,0], x[12,0]], [y[2,0], y[12,0]]) #3-13
    line37.set_3d_properties([z[2,0], z[12,0]])
    line38.set_data([x[12,0], x[14,0]], [y[12,0], y[14,0]]) #13-15
    line38.set_3d_properties([z[12,0], z[14,0]])
    line39.set_data([x[14,0], x[2,0]], [y[14,0], y[2,0]]) #15-3
    line39.set_3d_properties([z[14,0], z[2,0]])
    line40.set_data([x[16,0], x[15,0]], [y[16,0], y[15,0]]) #17-16
    line40.set_3d_properties([z[16,0], z[15,0]])
    line41.set_data([x[15,0], x[2,0]], [y[15,0], y[2,0]]) #16-3
    line41.set_3d_properties([z[15,0], z[2,0]])
    line42.set_data([x[2,0], x[16,0]], [y[2,0], y[16,0]]) #3-17
    line42.set_3d_properties([z[2,0], z[16,0]])
    line43.set_data([x[16,0], x[3,0]], [y[16,0], y[3,0]]) #17-4
    line43.set_3d_properties([z[16,0], z[3,0]])
    line44.set_data([x[3,0], x[17,0]], [y[3,0], y[17,0]]) #4-18
    line44.set_3d_properties([z[3,0], z[17,0]])
    line45.set_data([x[17,0], x[16,0]], [y[17,0], y[16,0]]) #18-17
    line45.set_3d_properties([z[17,0], z[16,0]])
    line46.set_data([x[0,0], x[15,0]], [y[0,0], y[15,0]]) #1-16
    line46.set_3d_properties([z[0,0], z[15,0]])
    line47.set_data([x[15,0], x[17,0]], [y[15,0], y[17,0]]) #16-18
    line47.set_3d_properties([z[15,0], z[17,0]])
    line48.set_data([x[17,0], x[0,0]], [y[17,0], y[0,0]]) #18-1
    line48.set_3d_properties([z[17,0], z[0,0]])
    line49.set_data([x[19,0], x[18,0]], [y[19,0], y[18,0]]) #20-19
    line49.set_3d_properties([z[19,0], z[18,0]])
    line50.set_data([x[18,0], x[10,0]], [y[18,0], y[10,0]]) #19-11
    line50.set_3d_properties([z[18,0], z[10,0]])
    line51.set_data([x[10,0], x[19,0]], [y[10,0], y[19,0]]) #11-20
    line51.set_3d_properties([z[10,0], z[19,0]])
    line52.set_data([x[19,0], x[11,0]], [y[19,0], y[11,0]]) #20-12
    line52.set_3d_properties([z[19,0], z[11,0]])
    line53.set_data([x[11,0], x[20,0]], [y[11,0], y[20,0]]) #12-21
    line53.set_3d_properties([z[11,0], z[20,0]])
    line54.set_data([x[20,0], x[19,0]], [y[20,0], y[19,0]]) #21-20
    line54.set_3d_properties([z[20,0], z[19,0]])
    line55.set_data([x[9,0], x[18,0]], [y[9,0], y[18,0]]) #10-19
    line55.set_3d_properties([z[9,0], z[18,0]])
    line56.set_data([x[18,0], x[20,0]], [y[18,0], y[20,0]]) #19-21
    line56.set_3d_properties([z[18,0], z[20,0]])
    line57.set_data([x[20,0], x[9,0]], [y[20,0], y[9,0]]) #21-10
    line57.set_3d_properties([z[20,0], z[9,0]])
    line58.set_data([x[22,0], x[21,0]], [y[22,0], y[21,0]]) #23-22
    line58.set_3d_properties([z[22,0], z[21,0]])
    line59.set_data([x[21,0], x[13,0]], [y[21,0], y[13,0]]) #22-14
    line59.set_3d_properties([z[21,0], z[13,0]])
    line60.set_data([x[13,0], x[22,0]], [y[13,0], y[22,0]]) #14-23
    line60.set_3d_properties([z[13,0], z[22,0]])
    line61.set_data([x[22,0], x[14,0]], [y[22,0], y[14,0]]) #23-15
    line61.set_3d_properties([z[22,0], z[14,0]])
    line62.set_data([x[14,0], x[23,0]], [y[14,0], y[23,0]]) #15-24
    line62.set_3d_properties([z[14,0], z[23,0]])
    line63.set_data([x[23,0], x[22,0]], [y[23,0], y[22,0]]) #24-23
    line63.set_3d_properties([z[23,0], z[22,0]])
    line64.set_data([x[12,0], x[21,0]], [y[12,0], y[21,0]]) #13-22
    line64.set_3d_properties([z[12,0], z[21,0]])
    line65.set_data([x[21,0], x[23,0]], [y[21,0], y[23,0]]) #22-24
    line65.set_3d_properties([z[21,0], z[23,0]])
    line66.set_data([x[23,0], x[12,0]], [y[23,0], y[12,0]]) #24-13
    line66.set_3d_properties([z[23,0], z[12,0]])
    line67.set_data([x[25,0], x[24,0]], [y[25,0], y[24,0]]) #26-25
    line67.set_3d_properties([z[25,0], z[24,0]])
    line68.set_data([x[24,0], x[7,0]], [y[24,0], y[7,0]]) #25-8
    line68.set_3d_properties([z[24,0], z[7,0]])
    line69.set_data([x[7,0], x[25,0]], [y[7,0], y[25,0]]) #8-26
    line69.set_3d_properties([z[7,0], z[25,0]])
    line70.set_data([x[25,0], x[8,0]], [y[25,0], y[8,0]]) #26-9
    line70.set_3d_properties([z[25,0], z[8,0]])
    line71.set_data([x[8,0], x[26,0]], [y[8,0], y[26,0]]) #9-27
    line71.set_3d_properties([z[8,0], z[26,0]])
    line72.set_data([x[26,0], x[25,0]], [y[26,0], y[25,0]]) #27-26
    line72.set_3d_properties([z[26,0], z[25,0]])
    line73.set_data([x[6,0], x[24,0]], [y[6,0], y[24,0]]) #7-25
    line73.set_3d_properties([z[6,0], z[24,0]])
    line74.set_data([x[24,0], x[26,0]], [y[24,0], y[26,0]]) #25-27
    line74.set_3d_properties([z[24,0], z[26,0]])
    line75.set_data([x[26,0], x[6,0]], [y[26,0], y[6,0]]) #27-7
    line75.set_3d_properties([z[26,0], z[6,0]])
    line76.set_data([x[28,0], x[27,0]], [y[28,0], y[27,0]]) #29-28
    line76.set_3d_properties([z[28,0], z[27,0]])
    line77.set_data([x[27,0], x[16,0]], [y[27,0], y[16,0]]) #28-17
    line77.set_3d_properties([z[27,0], z[16,0]])
    line78.set_data([x[16,0], x[28,0]], [y[16,0], y[28,0]]) #17-29
    line78.set_3d_properties([z[16,0], z[28,0]])
    line79.set_data([x[28,0], x[17,0]], [y[28,0], y[17,0]]) #29-18
    line79.set_3d_properties([z[28,0], z[17,0]])
    line80.set_data([x[17,0], x[29,0]], [y[17,0], y[29,0]]) #18-30
    line80.set_3d_properties([z[17,0], z[29,0]])
    line81.set_data([x[29,0], x[28,0]], [y[29,0], y[28,0]]) #30-29
    line81.set_3d_properties([z[29,0], z[28,0]])
    line82.set_data([x[15,0], x[27,0]], [y[15,0], y[27,0]]) #16-28
    line82.set_3d_properties([z[15,0], z[27,0]])
    line83.set_data([x[27,0], x[29,0]], [y[27,0], y[29,0]]) #28-30
    line83.set_3d_properties([z[27,0], z[29,0]])
    line84.set_data([x[29,0], x[15,0]], [y[29,0], y[15,0]]) #30-16
    line84.set_3d_properties([z[29,0], z[15,0]])

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
    node10.set_data(x[9], y[9])
    node10.set_3d_properties(z[9])
    node11.set_data(x[10], y[10])
    node11.set_3d_properties(z[10])
    node12.set_data(x[11], y[11])
    node12.set_3d_properties(z[11])
    node13.set_data(x[12], y[12])
    node13.set_3d_properties(z[12])
    node14.set_data(x[13], y[13])
    node14.set_3d_properties(z[13])
    node15.set_data(x[14], y[14])
    node15.set_3d_properties(z[14])
    node16.set_data(x[15], y[15])
    node16.set_3d_properties(z[15])
    node17.set_data(x[16], y[16])
    node17.set_3d_properties(z[16])
    node18.set_data(x[17], y[17])
    node18.set_3d_properties(z[17])
    node19.set_data(x[18], y[18])
    node19.set_3d_properties(z[18])
    node20.set_data(x[19], y[19])
    node20.set_3d_properties(z[19])
    node21.set_data(x[20], y[20])
    node21.set_3d_properties(z[20])
    node22.set_data(x[21], y[21])
    node22.set_3d_properties(z[21])
    node23.set_data(x[22], y[22])
    node23.set_3d_properties(z[22])
    node24.set_data(x[23], y[23])
    node24.set_3d_properties(z[23])
    node25.set_data(x[24], y[24])
    node25.set_3d_properties(z[24])
    node26.set_data(x[25], y[25])
    node26.set_3d_properties(z[25])
    node27.set_data(x[26], y[26])
    node27.set_3d_properties(z[26])
    node28.set_data(x[27], y[27])
    node28.set_3d_properties(z[27])
    node29.set_data(x[28], y[28])
    node29.set_3d_properties(z[28])
    node30.set_data(x[29], y[29])
    node30.set_3d_properties(z[29])


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
    node10_x.append(x[9])
    node10_y.append(y[9])
    node10_z.append(z[9])
    node11_x.append(x[10])
    node11_y.append(y[10])
    node11_z.append(z[10])
    node12_x.append(x[11])
    node12_y.append(y[11])
    node12_z.append(z[11])
    node13_x.append(x[12])
    node13_y.append(y[12])
    node13_z.append(z[12])
    node14_x.append(x[13])
    node14_y.append(y[13])
    node14_z.append(z[13])
    node15_x.append(x[14])
    node15_y.append(y[14])
    node15_z.append(z[14])
    node16_x.append(x[15])
    node16_y.append(y[15])
    node16_z.append(z[15])
    node17_x.append(x[16])
    node17_y.append(y[16])
    node17_z.append(z[16])
    node18_x.append(x[17])
    node18_y.append(y[17])
    node18_z.append(z[17])
    node19_x.append(x[18])
    node19_y.append(y[18])
    node19_z.append(z[18])
    node20_x.append(x[19])
    node20_y.append(y[19])
    node20_z.append(z[19])
    node21_x.append(x[20])
    node21_y.append(y[20])
    node21_z.append(z[20])
    node22_x.append(x[21])
    node22_y.append(y[21])
    node22_z.append(z[21])
    node23_x.append(x[22])
    node23_y.append(y[22])
    node23_z.append(z[22])
    node24_x.append(x[23])
    node24_y.append(y[23])
    node24_z.append(z[23])
    node25_x.append(x[24])
    node25_y.append(y[24])
    node25_z.append(z[24])
    node26_x.append(x[25])
    node26_y.append(y[25])
    node26_z.append(z[25])
    node27_x.append(x[26])
    node27_y.append(y[26])
    node27_z.append(z[26])
    node28_x.append(x[27])
    node28_y.append(y[27])
    node28_z.append(z[27])
    node29_x.append(x[28])
    node29_y.append(y[28])
    node29_z.append(z[28])
    node30_x.append(x[29])
    node30_y.append(y[29])
    node30_z.append(z[29])


    for idx, text in enumerate(node_texts):
        text.set_position((float(x[idx]), float(y[idx])))
        text.set_3d_properties(float(z[idx]), zdir='x')

    return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15, line16, line17, line18, line19, line20, line21, line22, line23, line24, line25, line26, line27, line28, line29, line30, line31, line32, line33, line34, line35, line36, line37, line38, line39, line40, line41, line42, line43, line44, line45, line46, line47, line48, line49, line50, line51, line52, line53, line54, line55, line56, line57, line58, line59, line60, line61, line62, line63, line64, line65, line66, line67, line68, line69, line70, line71, line72, line73, line74, line75, line76, line77, line78, line79, line80, line81, line82, line83, line84, node1, node2, node3, node4, node5, node6, node7, node8, node9, node10, node11, node12, node13, node14, node15, node16, node17, node18, node19, node20, node21, node22, node23, node24, node25, node26, node27, node28, node29, node30, *node_texts

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
# fig2, axs = plt.subplots(30, 3, figsize = (10, 12))
# axs[0, 0].set_title("X")
# axs[0, 1].set_title("Y")
# axs[0, 2].set_title("Z")
# axs[0, 0].set_ylabel("Node 1")
# axs[1, 0].set_ylabel("Node 2")
# axs[2, 0].set_ylabel("Node 3")
# axs[3, 0].set_ylabel("Node 4")
# axs[4, 0].set_ylabel("Node 5")
# axs[5, 0].set_ylabel("Node 6")
# axs[6, 0].set_ylabel("Node 7")
# axs[7, 0].set_ylabel("Node 8")
# axs[8, 0].set_ylabel("Node 9")
# axs[9, 0].set_ylabel("Node 10")
# axs[10, 0].set_ylabel("Node 11")
# axs[11, 0].set_ylabel("Node 12")
# axs[12, 0].set_ylabel("Node 13")
# axs[13, 0].set_ylabel("Node 14")
# axs[14, 0].set_ylabel("Node 15")
# axs[15, 0].set_ylabel("Node 16")
# axs[16, 0].set_ylabel("Node 17")
# axs[17, 0].set_ylabel("Node 18")
# axs[18, 0].set_ylabel("Node 19")
# axs[19, 0].set_ylabel("Node 20")
# axs[20, 0].set_ylabel("Node 21")
# axs[21, 0].set_ylabel("Node 22")
# axs[22, 0].set_ylabel("Node 23")
# axs[23, 0].set_ylabel("Node 24")
# axs[24, 0].set_ylabel("Node 25")
# axs[25, 0].set_ylabel("Node 26")
# axs[26, 0].set_ylabel("Node 27")
# axs[27, 0].set_ylabel("Node 28")
# axs[28, 0].set_ylabel("Node 29")
# axs[29, 0].set_ylabel("Node 30")


# for i in range(6):
#     for j in range(3):
#         axs[i, j].set_xlim(0, P.n_steps*P.Ts)
#         axs[i, j].set_ylim(-0.5, 2)

# axs[0, 0].plot(time, node1_x)
# axs[0, 1].plot(time, node1_y)
# axs[0, 2].plot(time, node1_z)
# axs[1, 0].plot(time, node2_x)
# axs[1, 1].plot(time, node2_y)
# axs[1, 2].plot(time, node2_z)
# axs[2, 0].plot(time, node3_x)
# axs[2, 1].plot(time, node3_y)
# axs[2, 2].plot(time, node3_z)
# axs[3, 0].plot(time, node4_x)
# axs[3, 1].plot(time, node4_y)
# axs[3, 2].plot(time, node4_z)
# axs[4, 0].plot(time, node5_x)
# axs[4, 1].plot(time, node5_y)
# axs[4, 2].plot(time, node5_z)
# axs[5, 0].plot(time, node6_x)
# axs[5, 1].plot(time, node6_y)
# axs[5, 2].plot(time, node6_z)
# axs[6, 0].plot(time, node7_x)
# axs[6, 1].plot(time, node7_y)
# axs[6, 2].plot(time, node7_z)
# axs[7, 0].plot(time, node8_x)
# axs[7, 1].plot(time, node8_y)
# axs[7, 2].plot(time, node8_z)
# axs[8, 0].plot(time, node9_x)
# axs[8, 1].plot(time, node9_y)
# axs[8, 2].plot(time, node9_z)
# axs[9, 0].plot(time, node10_x)
# axs[9, 1].plot(time, node10_y)
# axs[9, 2].plot(time, node10_z)
# axs[10, 0].plot(time, node11_x)
# axs[10, 1].plot(time, node11_y)
# axs[10, 2].plot(time, node11_z)
# axs[11, 0].plot(time, node12_x)
# axs[11, 1].plot(time, node12_y)
# axs[11, 2].plot(time, node12_z)
# axs[12, 0].plot(time, node13_x)
# axs[12, 1].plot(time, node13_y)
# axs[12, 2].plot(time, node13_z)
# axs[13, 0].plot(time, node14_x)
# axs[13, 1].plot(time, node14_y)
# axs[13, 2].plot(time, node14_z)
# axs[14, 0].plot(time, node15_x)
# axs[14, 1].plot(time, node15_y)
# axs[14, 2].plot(time, node15_z)
# axs[15, 0].plot(time, node16_x)
# axs[15, 1].plot(time, node16_y)
# axs[15, 2].plot(time, node16_z)
# axs[16, 0].plot(time, node17_x)
# axs[16, 1].plot(time, node17_y)
# axs[16, 2].plot(time, node17_z)
# axs[17, 0].plot(time, node18_x)
# axs[17, 1].plot(time, node18_y)
# axs[17, 2].plot(time, node18_z)
# axs[18, 0].plot(time, node19_x)
# axs[18, 1].plot(time, node19_y)
# axs[18, 2].plot(time, node19_z)
# axs[19, 0].plot(time, node20_x)
# axs[19, 1].plot(time, node20_y)
# axs[19, 2].plot(time, node20_z)
# axs[20, 0].plot(time, node21_x)
# axs[20, 1].plot(time, node21_y)
# axs[20, 2].plot(time, node21_z)
# axs[21, 0].plot(time, node22_x)
# axs[21, 1].plot(time, node22_y)
# axs[21, 2].plot(time, node22_z)
# axs[22, 0].plot(time, node23_x)
# axs[22, 1].plot(time, node23_y)
# axs[22, 2].plot(time, node23_z)
# axs[23, 0].plot(time, node24_x)
# axs[23, 1].plot(time, node24_y)
# axs[23, 2].plot(time, node24_z)
# axs[24, 0].plot(time, node25_x)
# axs[24, 1].plot(time, node25_y)
# axs[24, 2].plot(time, node25_z)
# axs[25, 0].plot(time, node26_x)
# axs[25, 1].plot(time, node26_y)
# axs[25, 2].plot(time, node26_z)
# axs[26, 0].plot(time, node27_x)
# axs[26, 1].plot(time, node27_y)
# axs[26, 2].plot(time, node27_z)
# axs[27, 0].plot(time, node28_x)
# axs[27, 1].plot(time, node28_y)
# axs[27, 2].plot(time, node28_z)
# axs[28, 0].plot(time, node29_x)
# axs[28, 1].plot(time, node29_y)
# axs[28, 2].plot(time, node29_z)
# axs[29, 0].plot(time, node30_x)
# axs[29, 1].plot(time, node30_y)
# axs[29, 2].plot(time, node30_z)

plt.show()