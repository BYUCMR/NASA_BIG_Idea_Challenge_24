import numpy as np
import octahedronParam3D as P
from RigidityMatrix3D import RigidityMatrix3D

class OctahedronDynamics:
    """
    This class outlines the dynamics of a single octahedron truss structure
    """
    # TODO: Add the ability to change the lengths of the tubes according to a kinematic solution
    # Needs to be implemented within the 'f' function, likely through the RM object

    def __init__(self):
        # Initial state condition. Requires x, y, z, and xdot, ydot, and zdot for each of the nodes
        # Order of the states: x, y, z for all nodes, followed by xdot, ydot, zdot for all nodes
        self.state = np.array([[P.x1],[P.y1],[P.z1],[P.x2],[P.y2],[P.z2],[P.x3],[P.y3],[P.z3],[P.x4],[P.y4],[P.z4],[P.x5],[P.y5],[P.z5],[P.x6],[P.y6],[P.z6],
                               [P.x1_dot],[P.y1_dot],[P.z1_dot],[P.x2_dot],[P.y2_dot],[P.z2_dot],[P.x3_dot],[P.y3_dot],[P.z3_dot],[P.x4_dot],[P.y4_dot],[P.z4_dot],[P.x5_dot],[P.y5_dot],[P.z5_dot],[P.x6_dot],[P.y6_dot],[P.z6_dot]])
        self.RM = RigidityMatrix3D()
        self.Ts = P.Ts
        self.mag = self.RM.Get_Lengths()

    def update(self, u):
        self.rk4_step(u)
        x = self.h()

    def f(self, state, tau = np.zeros((18))):
        '''
        Function to calculate the derivative of the state vector

        Inputs:
            state: np.array of the current states of the system
            tau: np.array of the external forces applied to the system
                Expects the order x, y, z for node 1, x, y, z for node 2, etc.
                Then the xdot, ydot, zdot for node 1, etc.

        Returns: 
            xdot: np.array of the derivative of the state vector
        '''
        # Rewrite the state vector to appear as a list of x, y, z coordinates & velocities for each node
        node1_pos = np.array([state[0][0], state[1][0], state[2][0]])
        node2_pos = np.array([state[3][0], state[4][0], state[5][0]])
        node3_pos = np.array([state[6][0], state[7][0], state[8][0]])
        node4_pos = np.array([state[9][0], state[10][0], state[11][0]])
        node5_pos = np.array([state[12][0], state[13][0], state[14][0]])
        node6_pos = np.array([state[15][0], state[16][0], state[17][0]])

        node1_vel = np.array([state[18][0], state[19][0], state[20][0]])
        node2_vel = np.array([state[21][0], state[22][0], state[23][0]])
        node3_vel = np.array([state[24][0], state[25][0], state[26][0]])
        node4_vel = np.array([state[27][0], state[28][0], state[29][0]])
        node5_vel = np.array([state[30][0], state[31][0], state[32][0]])
        node6_vel = np.array([state[33][0], state[34][0], state[35][0]])

        # TODO: Create code to identify which nodes are connected via tubes and use that to iteratively
        # construct the Fs and Fb vectors
        node_positions = np.array([node1_pos, node2_pos, node3_pos, node4_pos, node5_pos, node6_pos])
        node_velocities = np.array([node1_vel, node2_vel, node3_vel, node4_vel, node5_vel, node6_vel])

        # Additional variables for updating xdot at the end
        x2_dot = state[21][0]
        x3_dot = state[24][0]
        y3_dot = state[25][0]
        x4_dot = state[27][0]
        y4_dot = state[28][0]
        z4_dot = state[29][0]
        x5_dot = state[30][0]
        y5_dot = state[31][0]
        z5_dot = state[32][0]
        x6_dot = state[33][0]
        y6_dot = state[34][0]
        z6_dot = state[35][0]

        # TODO: Update the rigidity matrix based on the kinematics files (matlab or python)
        # Get the magnitudes of each side length
        x = np.array([[node1_pos[0], node2_pos[0], node3_pos[0], node4_pos[0], node5_pos[0], node6_pos[0]]])
        y = np.array([[node1_pos[1], node2_pos[1], node3_pos[1], node4_pos[1], node5_pos[1], node6_pos[1]]])
        z = np.array([[node1_pos[2], node2_pos[2], node3_pos[2], node4_pos[2], node5_pos[2], node6_pos[2]]])
        self.RM.x = np.hstack([np.transpose(x), np.transpose(y), np.transpose(z)])
        # mag = self.RM.Get_Lengths()
        # Get the unit vectors of each side
        R = self.RM.Get_R()
        # print(R)
        # Each row is one edge, every x component is grouped in the first six columns
        # Every y component is grouped in the next six columns
        # Every z component is grouped in the last six columns
        # Column 0 describes the x components of the unit vectors that end at node 1
        # Column 6 describes the y components of the unit vectors that end at node 1
        # Column 12 describes the z components of the unit vectors that end at node 1

        # Iteratively construct Fs for each tube (x12)
        Fs = np.zeros((12))
        Fs[0] = -P.k*(np.linalg.norm(node2_pos - node1_pos) - self.mag[0])
        Fs[1] = -P.k*(np.linalg.norm(node2_pos - node3_pos) - self.mag[1])
        Fs[2] = -P.k*(np.linalg.norm(node3_pos - node1_pos) - self.mag[2])
        Fs[3] = -P.k*(np.linalg.norm(node4_pos - node5_pos) - self.mag[3])
        Fs[4] = -P.k*(np.linalg.norm(node3_pos - node4_pos) - self.mag[4])
        Fs[5] = -P.k*(np.linalg.norm(node5_pos - node3_pos) - self.mag[5])
        Fs[6] = -P.k*(np.linalg.norm(node2_pos - node5_pos) - self.mag[6])
        Fs[7] = -P.k*(np.linalg.norm(node6_pos - node2_pos) - self.mag[7])
        Fs[8] = -P.k*(np.linalg.norm(node5_pos - node6_pos) - self.mag[8])
        Fs[9] = -P.k*(np.linalg.norm(node4_pos - node1_pos) - self.mag[9])
        Fs[10] = -P.k*(np.linalg.norm(node6_pos - node4_pos) - self.mag[10])
        Fs[11] = -P.k*(np.linalg.norm(node1_pos - node6_pos) - self.mag[11])

        # Construct Fb for each tube (subtract the current node's velocity from the other node's velocity)
        Fb = np.zeros((12))
        Fb[0] = -np.dot((node2_pos - node1_pos) / np.linalg.norm(node2_pos - node1_pos), node2_vel - node1_vel)*P.b
        Fb[1] = -np.dot((node2_pos - node3_pos) / np.linalg.norm(node2_pos - node3_pos), node2_vel - node3_vel)*P.b
        Fb[2] = -np.dot((node3_pos - node1_pos) / np.linalg.norm(node3_pos - node1_pos), node3_vel - node1_vel)*P.b
        Fb[3] = -np.dot((node4_pos - node5_pos) / np.linalg.norm(node4_pos - node5_pos), node4_vel - node5_vel)*P.b
        Fb[4] = -np.dot((node3_pos - node4_pos) / np.linalg.norm(node3_pos - node4_pos), node3_vel - node4_vel)*P.b
        Fb[5] = -np.dot((node5_pos - node3_pos) / np.linalg.norm(node5_pos - node3_pos), node5_vel - node3_vel)*P.b
        Fb[6] = -np.dot((node2_pos - node5_pos) / np.linalg.norm(node2_pos - node5_pos), node2_vel - node5_vel)*P.b
        Fb[7] = -np.dot((node6_pos - node2_pos) / np.linalg.norm(node6_pos - node2_pos), node6_vel - node2_vel)*P.b
        Fb[8] = -np.dot((node5_pos - node6_pos) / np.linalg.norm(node5_pos - node6_pos), node5_vel - node6_vel)*P.b
        Fb[9] = -np.dot((node4_pos - node1_pos) / np.linalg.norm(node4_pos - node1_pos), node4_vel - node1_vel)*P.b
        Fb[10] = -np.dot((node6_pos - node4_pos) / np.linalg.norm(node6_pos - node4_pos), node6_vel - node4_vel)*P.b
        Fb[11] = -np.dot((node1_pos - node6_pos) / np.linalg.norm(node1_pos - node6_pos), node1_vel - node6_vel)*P.b

        # Construct the force vector and add the external forces (18 item list of forces applied in the x/y/z directions to any of the nodes)
        # TODO: Include viscous friction against the ground for the bottom 3 nodes to reduce rotary oscillations
        Ff = np.zeros((12))
        factor = 2.0
        Ff[1] = node2_vel[0]*P.b*factor
        F = Fs + Fb + Ff
        gravity = P.g_vector    # List of the gravitational forces applied to each node in the -z directions

        sum_of_forces = F @ R + tau + gravity
        # Alternative equation can use R_T @ R
        
        # sum_of_forces will create a column vector of the sum of forces for nodes 1, 2, 3, 4, 5, and 6
        # in the x direction, followed by the y and z directions
        # Reconstruct the state vector using the equations of motion
        acc = (1 / P.m) * sum_of_forces
        x1ddot, x2ddot, x3ddot, x4ddot, x5ddot, x6ddot = acc[:6]
        y1ddot, y2ddot, y3ddot, y4ddot, y5ddot, y6ddot = acc[6:12]
        z1ddot, z2ddot, z3ddot, z4ddot, z5ddot, z6ddot = acc[12:]

        # Impose constraints on nodes 1, 2, and 3
        x1_dot = 0.0
        y1_dot = 0.0
        z1_dot = 0.0
        x2_dot = 0.0
        y2_dot = 0.0
        z2_dot = 0.0
        x3_dot = 0.0
        y3_dot = 0.0
        z3_dot = 0.0
        x1ddot = 0.0
        y1ddot = 0.0
        z1ddot = 0.0
        y2ddot = 0.0
        z2ddot = 0.0
        z3ddot = 0.0

        # Reconstruct the state vector
        xdot = np.array([[x1_dot], [y1_dot], [z1_dot], [x2_dot], [y2_dot], [z2_dot],
                         [x3_dot], [y3_dot], [z3_dot], [x4_dot], [y4_dot], [z4_dot],
                         [x5_dot], [y5_dot], [z5_dot], [x6_dot], [y6_dot], [z6_dot],
                         [x1ddot], [y1ddot], [z1ddot], [x2ddot], [y2ddot], [z2ddot],
                         [x3ddot], [y3ddot], [z3ddot], [x4ddot], [y4ddot], [z4ddot],
                         [x5ddot], [y5ddot], [z5ddot], [x6ddot], [y6ddot], [z6ddot]])
        return xdot
    
    def h(self):
        '''
        Function to collect and return the x, y, and z positions of the six nodes
        
        Returns:
            x: np.array of x positions of the six nodes (1, 2, 3, 4, 5, 6)
            y: np.array of y positions of the six nodes (1, 2, 3, 4, 5, 6)
            z: np.array of z positions of the six nodes (1, 2, 3, 4, 5, 6)
        '''
        x = np.zeros((6, 1))
        y = np.zeros((6, 1))
        z = np.zeros((6, 1))

        for i in range(6):
            x[i,0] = self.state[i*3][0]
            y[i,0] = self.state[i*3+1][0]
            z[i,0] = self.state[i*3+2][0]
        return x, y, z
    
    def rk4_step(self, tau):
        F1 = self.f(self.state, tau)
        F2 = self.f(self.state + self.Ts / 2.0 * F1, tau)
        F3 = self.f(self.state + self.Ts / 2.0 * F2, tau)
        F4 = self.f(self.state + self.Ts * F3, tau)
        self.state += self.Ts / 6.0 * (F1 + 2.0 * F2 + 2.0 * F3 + F4)

if __name__ == "__main__":
    import octahedron_1_sim



