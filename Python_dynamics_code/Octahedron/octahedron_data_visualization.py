import numpy as np
import matplotlib.pyplot as plt

# Load data
node1 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node0.csv', delimiter=',')
node2 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node1.csv', delimiter=',')
node3 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node2.csv', delimiter=',')

# plot the data in 3D space in two identical subplots
fig = plt.figure(figsize=(12, 6))

# First subplot
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot(node1[:,1], node1[:,2], node1[:,3], label='Node 1')
ax1.plot(node2[:,1], node2[:,2], node2[:,3], label='Node 2')
ax1.plot(node3[:,1], node3[:,2], node3[:,3], label='Node 3')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

# Add lines that follow the x, y, and z axes
ax1.plot([0, 1], [0, 0], [0, 0], color='r', linestyle='--', label='X-axis')
ax1.plot([0, 0], [0, 1], [0, 0], color='g', linestyle='--', label='Y-axis')
ax1.plot([0, 0], [0, 0], [0, 1], color='b', linestyle='--', label='Z-axis')

# Second subplot
ax2 = fig.add_subplot(122, projection='3d')
ax2.plot(node1[:,1], node1[:,2], node1[:,3], label='Node 1')
ax2.plot(node2[:,1], node2[:,2], node2[:,3], label='Node 2')
ax2.plot(node3[:,1], node3[:,2], node3[:,3], label='Node 3')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

# Add lines that follow the x, y, and z axes
ax2.plot([0, 1], [0, 0], [0, 0], color='r', linestyle='--', label='X-axis')
ax2.plot([0, 0], [0, 1], [0, 0], color='g', linestyle='--', label='Y-axis')
ax2.plot([0, 0], [0, 0], [0, 1], color='b', linestyle='--', label='Z-axis')

# start the time (index 0) at 0
node1[:, 0] = node1[:, 0] - node1[0, 0]
node2[:, 0] = node2[:, 0] - node2[0, 0]
node3[:, 0] = node3[:, 0] - node3[0, 0]

# calculate the vector from the original position of each node to the center of the truss
center = np.array([np.mean([node1[0,1], node2[0,1], node3[0,1]]), np.mean([node1[0,2], node2[0,2], node3[0,2]]), np.mean([node1[0,3], node2[0,3], node3[0,3]])])
ax1.plot(center[0], center[1], center[2], 'ro', label='Center of Mass')
ax2.plot(center[0], center[1], center[2], 'ro', label='Center of Mass')

# Specify legend location
# ax1.legend(loc='upper left', bbox_to_anchor=(-1, -1))

# Specify the z-axis limits
ax1.set_zlim(0, 2.0)
ax2.set_zlim(0, 2.0)

ax1.legend()
plt.show()

# map the data from each node onto the vector that points to the center of the truss
# from the original position of each node

# unit vector for node 1:
unit_vector0 = (center - node1[0,1:4]) / np.linalg.norm(center - node1[0,1:4])
# unit vector for node 2:
unit_vector1 = (center - node2[0,1:4]) / np.linalg.norm(center - node2[0,1:4])
# unit vector for node 3:
unit_vector2 = (center - node3[0,1:4]) / np.linalg.norm(center - node3[0,1:4])

# project the data from each node onto the unit vectors
node1_proj = np.dot(node1[:,1:4], unit_vector0)
node2_proj = np.dot(node2[:,1:4], unit_vector1)
node3_proj = np.dot(node3[:,1:4], unit_vector2)

node1_proj1 = np.dot(node1[:,1:4], unit_vector1)
node3_proj1 = np.dot(node3[:,1:4], unit_vector1)

node2_proj0 = np.dot(node2[:,1:4], unit_vector0)
node3_proj0 = np.dot(node3[:,1:4], unit_vector0)

node1_proj2 = np.dot(node1[:,1:4], unit_vector2)
node2_proj2 = np.dot(node2[:,1:4], unit_vector2)

# Center the data by subtracting the steady state value (first value)

node1_proj = node1_proj - node1_proj[0]
node2_proj = node2_proj - node2_proj[0]
node3_proj = node3_proj - node3_proj[0]

node1_proj1 = node1_proj1 - node1_proj1[0]
node3_proj1 = node3_proj1 - node3_proj1[0]

node2_proj0 = node2_proj0 - node2_proj0[0]
node3_proj0 = node3_proj0 - node3_proj0[0]

node1_proj2 = node1_proj2 - node1_proj2[0]
node2_proj2 = node2_proj2 - node2_proj2[0]

# plot the data on separate 2D plots in subplots
fig, axs = plt.subplots(2, 2, figsize=(10, 8))

# Plot node1 projections
axs[0, 0].plot(node1[:,0], node1_proj, label='node 1')
axs[0, 0].plot(node2[:,0], node2_proj0, label='node 2')
axs[0, 0].plot(node3[:,0], node3_proj0, label='node 3')
axs[0, 0].set_xlabel('Time')
axs[0, 0].set_ylabel('Displacement')
axs[0, 0].set_title(r'Node projections along $\hat{u}_{1,0}$')
axs[0, 0].legend()

# Plot node2 projections
axs[0, 1].plot(node1[:,0], node1_proj1, label='node 1')
axs[0, 1].plot(node2[:,0], node2_proj, label='node 2')
axs[0, 1].plot(node3[:,0], node3_proj1, label='node 3')
axs[0, 1].set_xlabel('Time')
axs[0, 1].set_title(r'Node projections along $\hat{u}_{2,0}$')
axs[0, 1].set_ylabel('Displacement')
axs[0, 1].legend()

# Plot node3 projections
axs[1, 0].plot(node2[:,0], node1_proj2, label='node 1')
axs[1, 0].plot(node1[:,0], node2_proj2, label='node 2')
axs[1, 0].plot(node3[:,0], node3_proj, label='node 3')
axs[1, 0].set_xlabel('Time')
axs[1, 0].set_title(r'Node projections along $\hat{u}_{3,0}$')
axs[1, 0].set_ylabel('Displacement')
axs[1, 0].legend()

# Plot all nodes projections together
axs[1, 1].plot(node1[:,0], node1_proj, label=r'node 1 dotted with $\hat{u}_{1-0}$')
axs[1, 1].plot(node2[:,0], node2_proj, label=r'node 2 dotted with $\hat{u}_{2-0}$')
axs[1, 1].plot(node3[:,0], node3_proj, label=r'node 3 dotted with $\hat{u}_{3-0}$')
axs[1, 1].set_xlabel('Time')
axs[1, 1].set_title('Node projections along their respective unit vectors')
axs[1, 1].set_ylabel('Displacement')
axs[1, 1].legend()

for ax in axs.flat:
    ax.set_xlim(4.0, 9.0)
    ax.set_ylim(-0.075, 0.125)

plt.tight_layout()
plt.show()

# Create a new figure with just the node1 projections
fig2, ax2 = plt.subplots(figsize=(6, 4))
ax2.plot(node1[:,0], node1_proj, label='node 1')
ax2.plot(node2[:,0], node2_proj0, label='node 2')
ax2.plot(node3[:,0], node3_proj0, label='node 3')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Displacement (m)')
# ax2.set_title(r'Node projections along $\hat{u}_{1,0}$')
ax2.legend()
ax2.set_xlim(4.0, 8.5)
ax2.set_ylim(-0.075, 0.125)
plt.tight_layout()
plt.show()

# find the maximum value of the projection between 4 and 5.5 s and store the index
max1a = np.max(node1_proj[np.logical_and(node1[:,0] >= 4, node1[:,0] <= 5.5)])
max2a = np.max(node2_proj0[np.logical_and(node2[:,0] >= 4, node2[:,0] <= 5.5)])
max3a = np.max(node3_proj0[np.logical_and(node3[:,0] >= 4, node3[:,0] <= 5.5)])

# find the time at which the maximum value occurs
time1a = node1[np.where(node1_proj == max1a)[0][0], 0]
time2a = node2[np.where(node2_proj0 == max2a)[0][0], 0]
time3a = node3[np.where(node3_proj0 == max3a)[0][0], 0]

# find the maximum value of the projection between 8.6 and 8.95 s and store the index
max1b = np.max(node1_proj[np.logical_and(node1[:,0] >= 8.6, node1[:,0] <= 8.95)])
max2b = np.max(node2_proj0[np.logical_and(node2[:,0] >= 8.6, node2[:,0] <= 8.95)])
max3b = np.max(node3_proj0[np.logical_and(node3[:,0] >= 8.6, node3[:,0] <= 8.95)])

# find the time at which the maximum value occurs
time1b = node1[np.where(node1_proj == max1b)[0][0], 0]
time2b = node2[np.where(node2_proj0 == max2b)[0][0], 0]
time3b = node3[np.where(node3_proj0 == max3b)[0][0], 0]

n = 5
# log decrement method
delta = (1/n) * np.log(max1a / max1b)
zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)
T = (time1b - time1a) / n
omega_d = 2 * np.pi / T
omega_n = omega_d / np.sqrt(1 - zeta**2)
print(f"node 1 projections: zeta = {zeta}")
print(f"node 1 projections: omega_d = {omega_d}")
print(f"node 1 projections: omega_n = {omega_n}")

delta = (1/n) * np.log(max2a / max2b)
zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)
T = (time2b - time2a) / n
omega_d = 2 * np.pi / T
omega_n = omega_d / np.sqrt(1 - zeta**2)
print(f"node 2 projections: zeta = {zeta}")
print(f"node 2 projections: omega_d = {omega_d}")
print(f"node 2 projections: omega_n = {omega_n}")

delta = (1/n) * np.log(max3a / max3b)
zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)
T = (time3b - time3a) / n
omega_d = 2 * np.pi / T
omega_n = omega_d / np.sqrt(1 - zeta**2)
print(f"node 3 projections: zeta = {zeta}")
print(f"node 3 projections: omega_d = {omega_d}")
print(f"node 3 projections: omega_n = {omega_n}")

print("")