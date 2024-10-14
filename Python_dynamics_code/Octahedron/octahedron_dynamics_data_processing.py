import rosbag
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
import matplotlib
from sklearn.decomposition import PCA
import time
matplotlib.use('TkAgg')
def read_pose_messages(bag, topic, num_messages):
    ret_mat = np.zeros((9, num_messages))
    counter = 0
    loop = tqdm(total=num_messages, position=0, leave=False)
    for topic, msg, t in bag.read_messages(topic):
        loop.set_description('Processing {0} messages: {1} out of {2}'.format(topic, counter, num_messages))
        loop.update(1)
        header_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        bag_time = float(str(t)) * 1e-9
        Px = msg.pose.position.x
        Py = msg.pose.position.y
        Pz = msg.pose.position.z
        Qx = msg.pose.orientation.x
        Qy = msg.pose.orientation.y
        Qz = msg.pose.orientation.z
        Qw = msg.pose.orientation.w
        ret_mat[:, counter] = np.array([header_time, bag_time, Px, Pz, Py, Qx, Qy, Qz, Qw])
        counter = counter + 1
    loop.close()
    return ret_mat

def plot_coordinates(ret_mat_0, ret_mat_1, ret_mat_2):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot pose_0
    Px_0 = ret_mat_0[2, :]
    Py_0 = ret_mat_0[3, :]
    Pz_0 = ret_mat_0[4, :]
    ax.plot(Px_0, Py_0, Pz_0, label='Pose 0')

    # Plot pose_1
    Px_1 = ret_mat_1[2, :]
    Py_1 = ret_mat_1[3, :]
    Pz_1 = ret_mat_1[4, :]
    ax.plot(Px_1, Py_1, Pz_1, label='Pose 1')

    # Plot pose_2
    Px_2 = ret_mat_2[2, :]
    Py_2 = ret_mat_2[3, :]
    Pz_2 = ret_mat_2[4, :]
    ax.plot(Px_2, Py_2, Pz_2, label='Pose 2')

    ax.set_xlabel('Px')
    ax.set_ylabel('Py')
    ax.set_zlabel('Pz')
    ax.set_title('3D Position Coordinates')
    ax.legend()

    # Set axis limits
    ax.set_xlim([0, 3])
    ax.set_ylim([0, 3])
    ax.set_zlim([0, 2])

    plt.show()

octahedron_bag = rosbag.Bag('/home/spencerstowell/Documents/ResearchProjects/code_for_soft_robots_Usevitch/Python_dynamics_code/Octahedron/Octahedron.bag')

# Read messages for pose_0
num_messages_0 = octahedron_bag.get_message_count("/vive/pose_0")
ret_mat_0 = read_pose_messages(octahedron_bag, '/vive/pose_0', num_messages_0)

# Read messages for pose_1
num_messages_1 = octahedron_bag.get_message_count("/vive/pose_1")
ret_mat_1 = read_pose_messages(octahedron_bag, '/vive/pose_1', num_messages_1)

# Read messages for pose_2
num_messages_2 = octahedron_bag.get_message_count("/vive/pose_2")
ret_mat_2 = read_pose_messages(octahedron_bag, '/vive/pose_2', num_messages_2)

# Combine all points from the three poses
all_points = np.hstack((ret_mat_0[2:5, :], ret_mat_1[2:5, :], ret_mat_2[2:5, :]))

# Perform PCA to find the plane of best fit
pca = PCA(n_components=2)
pca.fit(all_points.T)

# The plane of best fit is defined by the first two principal components
plane_normal = np.cross(pca.components_[0], pca.components_[1])

# Create a transformation matrix to align the plane of best fit with the XY plane
# The transformation matrix will rotate the plane normal to align with the Z axis
def create_transformation_matrix(normal):
    z_axis = np.array([0, 0, 1])
    v = np.cross(normal, z_axis)
    c = np.dot(normal, z_axis)
    k = 1 / (1 + c)
    return np.array([
        [v[0] * v[0] * k + c, v[0] * v[1] * k - v[2], v[0] * v[2] * k + v[1]],
        [v[1] * v[0] * k + v[2], v[1] * v[1] * k + c, v[1] * v[2] * k - v[0]],
        [v[2] * v[0] * k - v[1], v[2] * v[1] * k + v[0], v[2] * v[2] * k + c]
    ])

transformation_matrix = create_transformation_matrix(plane_normal)

# Apply the transformation matrix to all points
def apply_transformation(ret_mat, transformation_matrix):
    points = ret_mat[2:5, :]
    transformed_points = transformation_matrix @ points
    ret_mat[2:5, :] = transformed_points

apply_transformation(ret_mat_0, transformation_matrix)
apply_transformation(ret_mat_1, transformation_matrix)
apply_transformation(ret_mat_2, transformation_matrix)

# add 6 meters to the y coordinate of all poses
ret_mat_0[3, :] = ret_mat_0[3, :] + 6
ret_mat_1[3, :] = ret_mat_1[3, :] + 6
ret_mat_2[3, :] = ret_mat_2[3, :] + 6

# add 3 meters to the z coordinate of all poses
ret_mat_0[4, :] = ret_mat_0[4, :] + 3
ret_mat_1[4, :] = ret_mat_1[4, :] + 3
ret_mat_2[4, :] = ret_mat_2[4, :] + 3

# subtract 0.75 meter from the x coordinate of all poses
ret_mat_0[2, :] = ret_mat_0[2, :] - 0.75
ret_mat_1[2, :] = ret_mat_1[2, :] - 0.75
ret_mat_2[2, :] = ret_mat_2[2, :] - 0.75

# Plot all poses on the same 3D figure
plot_coordinates(ret_mat_0, ret_mat_1, ret_mat_2)

# export the data to a single csv file
np.savetxt('octahedron_data.csv', np.hstack((ret_mat_0, ret_mat_1, ret_mat_2)), delimiter=',')
octahedron_bag.close()

# def animate_coordinates(ret_mat_0, ret_mat_1, ret_mat_2):
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     num_points = ret_mat_0.shape[1]
#     for i in range(num_points):
#         ax.cla()  # Clear the axes for the next frame

#         # Plot pose_0
#         ax.scatter(ret_mat_0[2, i], ret_mat_0[3, i], ret_mat_0[4, i], color='r', label='Pose 0' if i == 0 else "")

#         # Plot pose_1
#         ax.scatter(ret_mat_1[2, i], ret_mat_1[3, i], ret_mat_1[4, i], color='g', label='Pose 1' if i == 0 else "")

#         # Plot pose_2
#         ax.scatter(ret_mat_2[2, i], ret_mat_2[3, i], ret_mat_2[4, i], color='b', label='Pose 2' if i == 0 else "")

#         ax.set_xlabel('Px')
#         ax.set_ylabel('Py')
#         ax.set_zlabel('Pz')
#         ax.set_title('Time-dependent 3D Position Coordinates')
#         ax.set_xlim([0, 3])
#         ax.set_ylim([0, 3])  # Adjusted for the y-coordinate shift
#         ax.set_zlim([1.5, 2])   # Adjusted for the z-coordinate shift
#         ax.legend()

#         plt.draw()
#         # print(f"Frame {i + 1} of {num_points}")
#         plt.pause(1/800)  # Pause to create animation effect

#     plt.show()

# animate_coordinates(ret_mat_0, ret_mat_1, ret_mat_2)
