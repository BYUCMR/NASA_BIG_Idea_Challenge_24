import numpy as np
import matplotlib.pyplot as plt

class OctahedronDataVisualization:
    def __init__(self):
        self.node_data = []
        for i in range(3):
            node_data = np.loadtxt(f'Python_dynamics_code/Octahedron/octahedron_data_node{i}.csv', delimiter=',')
            node_data[:, 0] -= node_data[0, 0]  # set the time (index 0) to 0 for all nodes
            self.node_data.append(node_data)
        
        self.node_data = np.array(self.node_data)

        self.center = np.array([np.mean(self.node_data[:,0,1]), np.mean(self.node_data[:,0,2]), np.mean(self.node_data[:,0,3])])
        self.generate_node_projections()
        self.get_max_value_and_time()

    def generate_node_projections(self):
        unit_vectors = np.zeros((3, 3))
        for i in range(3):
            unit_vectors[i] = (self.center - self.node_data[i][0,1:4]) / np.linalg.norm(self.center - self.node_data[i][0,1:4])

        self.projections = np.zeros((3, self.node_data[0].shape[0], 3))
        for i in range(3):
            for j in range(3):
                self.projections[i, :, j] = np.dot(self.node_data[i][:, 1:4], unit_vectors[j])
                self.projections[i, :, j] -= self.projections[i, 0, j]  # Center the data by subtracting the steady-state value (first value)

    def plot_3D(self):
        # plot the data in 3D space in two identical subplots
        fig, axes = plt.subplots(1, 2, subplot_kw={'projection': '3d'}, figsize=(12, 6))
            
        length = 1.0
        for ax in axes:
            for i, label in enumerate(['Node 1', 'Node 2', 'Node 3']):
                ax.plot(self.node_data[i][:, 1], self.node_data[i][:, 2], self.node_data[i][:, 3], label=label)
            ax.plot(self.center[0], self.center[1], self.center[2], 'ro', label='Center of Mass')
            ax.plot([0, length], [0, 0], [0, 0], color='r', linestyle='--')
            ax.plot([0, 0], [0, length], [0, 0], color='g', linestyle='--')
            ax.plot([0, 0], [0, 0], [0, length], color='b', linestyle='--')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_zlim(0, 2.0)
            ax.legend()

        plt.show()

    def plot_node_projection(self, node_index=0):
        fig, ax = plt.subplots(figsize=(6, 4))

        ax.plot(self.node_data[0][:, 0], self.projections[0][:, node_index], label='node 1')
        ax.plot(self.node_data[1][:, 0], self.projections[1][:, node_index], label='node 2')
        ax.plot(self.node_data[2][:, 0], self.projections[2][:, node_index], label='node 3')

        plt.xlabel('Time (s)')
        plt.ylabel('Displacement (m)')
        ax.legend()
        ax.set_xlim(4.0, 9.0)
        ax.set_ylim(-0.075, 0.125)
        plt.tight_layout()
        plt.show()

    def log_decrement(self, n=5):
        self.node_characteristics = []

        for node_index in range(len(self.all_max_values)):
            # Calculate the logarithmic decrement (delta)
            max_value_1 = self.all_max_values[node_index][0]
            max_value_2 = self.all_max_values[node_index][1]
            delta = (1 / n) * np.log(max_value_1 / max_value_2)

            # Calculate the damping ratio (zeta)
            zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)

            # Calculate the time period (T) and frequencies
            time_interval = self.all_times[node_index][1] - self.all_times[node_index][0]
            T = time_interval / n
            omega_d = 2 * np.pi / T  # Damped natural frequency
            omega_n = omega_d / np.sqrt(1 - zeta**2)  # Natural frequency

            # Store the results for the current node
            self.node_characteristics.append((zeta, omega_d, omega_n))
    
    def get_max_value_and_time(self, start_times=[4, 8.6], end_times=[5.5, 8.95]):
        self.all_max_values, self.all_times = [], []

        for node_index in range(3):  # Loop over all nodes
            max_values, times = [], []

            for interval_index in range(2):
                start_time = start_times[interval_index]
                end_time = end_times[interval_index]

                mask = (start_time <= self.node_data[node_index][:, 0]) & (self.node_data[node_index][:, 0] <= end_time)
                max_value = self.projections[node_index][mask, 0].max()
                max_time = self.node_data[node_index][np.where(self.projections[node_index][:, 0] == max_value)[0][0], 0]

                max_values.append(max_value)
                times.append(max_time)

            self.all_max_values.append(max_values)
            self.all_times.append(times)

OctDataViz = OctahedronDataVisualization()
# OctDataViz.plot_3D()
OctDataViz.plot_node_projection()
# OctDataViz.log_decrement()
# print("Node characteristics:")
# for i, (zeta, omega_d, omega_n) in enumerate(OctDataViz.node_characteristics):
#     print(f"Node {i+1}: zeta = {zeta}, omega_d = {omega_d}, omega_n = {omega_n}")