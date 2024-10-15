import numpy as np
import matplotlib.pyplot as plt

# Load data
node0 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node0.csv', delimiter=',')
node1 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node1.csv', delimiter=',')
node2 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node2.csv', delimiter=',')

# plot the data in 3D space
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(node0[:,1], node0[:,2], node0[:,3], label='Node 0')
ax.plot(node1[:,1], node1[:,2], node1[:,3], label='Node 1')
ax.plot(node2[:,1], node2[:,2], node2[:,3], label='Node 2')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# start the time (index 0) at 0
node0[:, 0] = node0[:, 0] - node0[0, 0]
node1[:, 0] = node1[:, 0] - node1[0, 0]
node2[:, 0] = node2[:, 0] - node2[0, 0]

# calculate the vector from the original position of each node to the center of the truss
center = np.array([np.mean([node0[0,1], node1[0,1], node2[0,1]]), np.mean([node0[0,2], node1[0,2], node2[0,2]]), np.mean([node0[0,3], node1[0,3], node2[0,3]])])
ax.plot(center[0], center[1], center[2], 'ro', label='Center of Truss')

#Specify the z-axis limits
ax.set_zlim(0, 2.0)
ax.legend()
plt.show()

# map the data from each node onto the vector that points to the center of the truss
# from the original position of each node

# unit vector for node 0:
unit_vector0 = (center - node0[0,1:4]) / np.linalg.norm(center - node0[0,1:4])
# unit vector for node 1:
unit_vector1 = (center - node1[0,1:4]) / np.linalg.norm(center - node1[0,1:4])
# unit vector for node 2:
unit_vector2 = (center - node2[0,1:4]) / np.linalg.norm(center - node2[0,1:4])

# project the data from each node onto the unit vectors
node0_proj = np.dot(node0[:,1:4], unit_vector0)
node1_proj = np.dot(node1[:,1:4], unit_vector1)
node2_proj = np.dot(node2[:,1:4], unit_vector2)

node0_proj1 = np.dot(node0[:,1:4], unit_vector1)
node2_proj1 = np.dot(node2[:,1:4], unit_vector1)

node1_proj0 = np.dot(node1[:,1:4], unit_vector0)
node2_proj0 = np.dot(node2[:,1:4], unit_vector0)

node0_proj2 = np.dot(node0[:,1:4], unit_vector2)
node1_proj2 = np.dot(node1[:,1:4], unit_vector2)

# Center the data by subtracting the steady state value (first value)

node0_proj = node0_proj - node0_proj[0]
node1_proj = node1_proj - node1_proj[0]
node2_proj = node2_proj - node2_proj[0]

node0_proj1 = node0_proj1 - node0_proj1[0]
node2_proj1 = node2_proj1 - node2_proj1[0]

node1_proj0 = node1_proj0 - node1_proj0[0]
node2_proj0 = node2_proj0 - node2_proj0[0]

node0_proj2 = node0_proj2 - node0_proj2[0]
node1_proj2 = node1_proj2 - node1_proj2[0]

# plot the data on separate 2D plots in subplots
fig, axs = plt.subplots(2, 2, figsize=(10, 8))

# Plot node0 projections
axs[0, 0].plot(node0[:,0], node0_proj, label='Node 0')
axs[0, 0].plot(node1[:,0], node1_proj0, label='Node 1 projected along 0')
axs[0, 0].plot(node2[:,0], node2_proj0, label='Node 2 projected along 0')
axs[0, 0].set_xlabel('Time')
axs[0, 0].set_ylabel('Displacement')
axs[0, 0].legend()

# Plot node1 projections
axs[0, 1].plot(node1[:,0], node1_proj, label='Node 1')
axs[0, 1].plot(node0[:,0], node0_proj1, label='Node 0 projected along 1')
axs[0, 1].plot(node2[:,0], node2_proj1, label='Node 2 projected along 1')
axs[0, 1].set_xlabel('Time')

axs[0, 1].set_ylabel('Displacement')
axs[0, 1].legend()

# Plot node2 projections
axs[1, 0].plot(node2[:,0], node2_proj, label='Node 2')
axs[1, 0].plot(node1[:,0], node0_proj2, label='Node 0 projected along 2')
axs[1, 0].plot(node0[:,0], node1_proj2, label='Node 1 projected along 2')
axs[1, 0].set_xlabel('Time')
axs[1, 0].set_ylabel('Displacement')
axs[1, 0].legend()

# Plot all nodes projections together
axs[1, 1].plot(node0[:,0], node0_proj, label='Node 0')
axs[1, 1].plot(node1[:,0], node1_proj, label='Node 1')
axs[1, 1].plot(node2[:,0], node2_proj, label='Node 2')
axs[1, 1].set_xlabel('Time')
axs[1, 1].set_ylabel('Displacement')
axs[1, 1].legend()

for ax in axs.flat:
    ax.set_xlim(0, 36.2)

plt.tight_layout()
plt.show()

# find the maximum value of the projection between 4 and 5.5 s and store the index
max0a = np.max(node0_proj[np.logical_and(node0[:,0] >= 4, node0[:,0] <= 5.5)])
max1a = np.max(node1_proj0[np.logical_and(node1[:,0] >= 4, node1[:,0] <= 5.5)])
max2a = np.max(node2_proj0[np.logical_and(node2[:,0] >= 4, node2[:,0] <= 5.5)])

# find the time at which the maximum value occurs
time0a = node0[np.where(node0_proj == max0a)[0][0], 0]
time1a = node1[np.where(node1_proj0 == max1a)[0][0], 0]
time2a = node2[np.where(node2_proj0 == max2a)[0][0], 0]

# find the maximum value of the projection between 8.6 and 8.95 s and store the index
max0b = np.max(node0_proj[np.logical_and(node0[:,0] >= 8.6, node0[:,0] <= 8.95)])
max1b = np.max(node1_proj0[np.logical_and(node1[:,0] >= 8.6, node1[:,0] <= 8.95)])
max2b = np.max(node2_proj0[np.logical_and(node2[:,0] >= 8.6, node2[:,0] <= 8.95)])

# find the time at which the maximum value occurs
time0b = node0[np.where(node0_proj == max0b)[0][0], 0]
time1b = node1[np.where(node1_proj0 == max1b)[0][0], 0]
time2b = node2[np.where(node2_proj0 == max2b)[0][0], 0]

n = 5
# log decrement method
delta = (1/n) * np.log(max0a / max0b)
zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)
omega_d = 2 * np.pi / (time0b - time0a)
print(f"Node 0 projections: zeta = {zeta}")
print(f"Node 0 projections: omega_d = {omega_d}")

delta = (1/n) * np.log(max1a / max1b)
zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)
omega_d = 2 * np.pi / (time1b - time1a)
print(f"Node 1 projections: zeta = {zeta}")
print(f"Node 1 projections: omega_d = {omega_d}")

delta = (1/n) * np.log(max2a / max2b)
zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)
omega_d = 2 * np.pi / (time2b - time2a)
print(f"Node 2 projections: zeta = {zeta}")
print(f"Node 2 projections: omega_d = {omega_d}")

print("")

