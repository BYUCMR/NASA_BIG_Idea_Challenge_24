import numpy as np
import solar_array_param as P
from solar_array_rigidity_matrix import SolarRigidityMatrix

class SolarDynamics:
    """
    This class outlines the dynamics of a single octahedron truss structure
    """
    # TODO: Add the ability to change the lengths of the tubes according to a kinematic solution
    # Needs to be implemented within the 'f' function, likely through the RM object

    def __init__(self):
        # Initial state condition. Requires x, y, z, and xdot, ydot, and zdot for each of the nodes
        # Order of the states: x, y, z for all nodes, followed by xdot, ydot, zdot for all nodes
        self.state = np.array([[P.x1],[P.y1],[P.z1],[P.x2],[P.y2],[P.z2],[P.x3],[P.y3],[P.z3],
                               [P.x4],[P.y4],[P.z4],[P.x5],[P.y5],[P.z5],[P.x6],[P.y6],[P.z6],
                               [P.x7],[P.y7],[P.z7],[P.x8],[P.y8],[P.z8],[P.x9],[P.y9],[P.z9],
                               [P.x1_dot],[P.y1_dot],[P.z1_dot],[P.x2_dot],[P.y2_dot],[P.z2_dot],[P.x3_dot],[P.y3_dot],[P.z3_dot],
                               [P.x4_dot],[P.y4_dot],[P.z4_dot],[P.x5_dot],[P.y5_dot],[P.z5_dot],[P.x6_dot],[P.y6_dot],[P.z6_dot],
                               [P.x7_dot],[P.y7_dot],[P.z7_dot],[P.x8_dot],[P.y8_dot],[P.z8_dot],[P.x9_dot],[P.y9_dot],[P.z9_dot]])
        self.RM = SolarRigidityMatrix()
        self.Ts = P.Ts

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
        node7_pos = np.array([state[18][0], state[19][0], state[20][0]])
        node8_pos = np.array([state[21][0], state[22][0], state[23][0]])
        node9_pos = np.array([state[24][0], state[25][0], state[26][0]])

        node1_vel = np.array([state[27][0], state[28][0], state[29][0]])
        node2_vel = np.array([state[30][0], state[31][0], state[32][0]])
        node3_vel = np.array([state[33][0], state[34][0], state[35][0]])
        node4_vel = np.array([state[36][0], state[37][0], state[38][0]])
        node5_vel = np.array([state[39][0], state[40][0], state[41][0]])
        node6_vel = np.array([state[42][0], state[43][0], state[44][0]])
        node7_vel = np.array([state[45][0], state[46][0], state[47][0]])
        node8_vel = np.array([state[48][0], state[49][0], state[50][0]])
        node9_vel = np.array([state[51][0], state[52][0], state[53][0]])
        

        # TODO: Create code to identify which nodes are connected via tubes and use that to iteratively
        # construct the Fs and Fb vectors
        node_positions = np.array([node1_pos, node2_pos, node3_pos, node4_pos, node5_pos, node6_pos, node7_pos, node8_pos, node9_pos])
        node_velocities = np.array([node1_vel, node2_vel, node3_vel, node4_vel, node5_vel, node6_vel, node7_vel, node8_vel, node9_vel])

        # Additional variables for updating xdot at the end
        x2_dot = state[30][0]
        x3_dot = state[33][0]
        y3_dot = state[34][0]
        x4_dot = state[36][0]
        y4_dot = state[37][0]
        z4_dot = state[38][0]
        x5_dot = state[39][0]
        y5_dot = state[40][0]
        z5_dot = state[41][0]
        x6_dot = state[42][0]
        y6_dot = state[43][0]
        z6_dot = state[44][0]
        x7_dot = state[45][0]
        y7_dot = state[46][0]
        z7_dot = state[47][0]
        x8_dot = state[48][0]
        y8_dot = state[49][0]
        z8_dot = state[50][0]
        x9_dot = state[51][0]
        y9_dot = state[52][0]
        z9_dot = state[53][0]

        # TODO: Update the rigidity matrix based on the kinematics files (matlab or python)
        # Get the magnitudes of each side length
        mag = self.RM.Get_Lengths()
        # Get the unit vectors of each side
        R = self.RM.Get_R()
        # Each row is one edge, every x component is grouped in the first six columns
        # Every y component is grouped in the next six columns
        # Every z component is grouped in the last six columns
        # Column 0 describes the x components of the unit vectors that end at node 1
        # Column 6 describes the y components of the unit vectors that end at node 1
        # Column 12 describes the z components of the unit vectors that end at node 1

        # Iteratively construct Fs for each tube (x12)
        Fs = np.zeros((21))
        Fs[0] = -P.k*(np.linalg.norm(node2_pos - node1_pos) - mag[0])
        Fs[1] = -P.k*(np.linalg.norm(node3_pos - node2_pos) - mag[1])
        Fs[2] = -P.k*(np.linalg.norm(node3_pos - node1_pos) - mag[2])
        Fs[3] = -P.k*(np.linalg.norm(node4_pos - node5_pos) - mag[3])
        Fs[4] = -P.k*(np.linalg.norm(node3_pos - node4_pos) - mag[4])
        Fs[5] = -P.k*(np.linalg.norm(node5_pos - node3_pos) - mag[5])
        Fs[6] = -P.k*(np.linalg.norm(node2_pos - node5_pos) - mag[6])
        Fs[7] = -P.k*(np.linalg.norm(node6_pos - node2_pos) - mag[7])
        Fs[8] = -P.k*(np.linalg.norm(node5_pos - node6_pos) - mag[8])
        Fs[9] = -P.k*(np.linalg.norm(node4_pos - node1_pos) - mag[9])
        Fs[10] = -P.k*(np.linalg.norm(node6_pos - node4_pos) - mag[10])
        Fs[11] = -P.k*(np.linalg.norm(node1_pos - node6_pos) - mag[11])
        Fs[12] = -P.k*(np.linalg.norm(node7_pos - node8_pos) - mag[12])
        Fs[13] = -P.k*(np.linalg.norm(node6_pos - node7_pos) - mag[13])
        Fs[14] = -P.k*(np.linalg.norm(node8_pos - node6_pos) - mag[14])
        Fs[15] = -P.k*(np.linalg.norm(node4_pos - node8_pos) - mag[15])
        Fs[16] = -P.k*(np.linalg.norm(node9_pos - node4_pos) - mag[16])
        Fs[17] = -P.k*(np.linalg.norm(node8_pos - node9_pos) - mag[17])
        Fs[18] = -P.k*(np.linalg.norm(node7_pos - node5_pos) - mag[18])
        Fs[19] = -P.k*(np.linalg.norm(node9_pos - node7_pos) - mag[19])
        Fs[20] = -P.k*(np.linalg.norm(node5_pos - node9_pos) - mag[20])

        # Construct Fb for each tube (subtract the current node's velocity from the other node's velocity)
        Fb = np.zeros((21))
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
        Fb[12] = -np.dot((node7_pos - node8_pos) / np.linalg.norm(node7_pos - node8_pos), node7_vel - node8_vel)*P.b
        Fb[13] = -np.dot((node6_pos - node7_pos) / np.linalg.norm(node6_pos - node7_pos), node6_vel - node7_vel)*P.b
        Fb[14] = -np.dot((node8_pos - node6_pos) / np.linalg.norm(node8_pos - node6_pos), node8_vel - node6_vel)*P.b
        Fb[15] = -np.dot((node4_pos - node8_pos) / np.linalg.norm(node4_pos - node8_pos), node4_vel - node8_vel)*P.b
        Fb[16] = -np.dot((node9_pos - node4_pos) / np.linalg.norm(node9_pos - node4_pos), node9_vel - node4_vel)*P.b
        Fb[17] = -np.dot((node8_pos - node9_pos) / np.linalg.norm(node8_pos - node9_pos), node8_vel - node9_vel)*P.b
        Fb[18] = -np.dot((node7_pos - node5_pos) / np.linalg.norm(node7_pos - node5_pos), node7_vel - node5_vel)*P.b
        Fb[19] = -np.dot((node9_pos - node7_pos) / np.linalg.norm(node9_pos - node7_pos), node9_vel - node7_vel)*P.b
        Fb[20] = -np.dot((node5_pos - node9_pos) / np.linalg.norm(node5_pos - node9_pos), node5_vel - node9_vel)*P.b

        # Construct the force vector and add the external forces (18 item list of forces applied in the x/y/z directions to any of the nodes)
        # TODO: Include viscous friction against the ground for the bottom 3 nodes to reduce rotary oscillations
        Ff = np.zeros((21))
        factor = 2.0
        Ff[1] = node2_vel[0]*P.b*factor
        F = Fs + Fb + Ff
        gravity = P.g_vector    # List of the gravitational forces applied to each node in the -z directions

        sum_of_forces = F @ R + tau + gravity
        # Alternative equation can use R_T @ R
        
        # sum_of_forces will create a column vector of the sum of forces for nodes 1, 2, 3, 4, 5, and 6
        # in the x direction, followed by the y and z directions
        # Reconstruct the state vector using the equations of motion
        x1ddot = (1/P.m)*(sum_of_forces[0])
        x2ddot = (1/P.m)*(sum_of_forces[1])
        x3ddot = (1/P.m)*(sum_of_forces[2])
        x4ddot = (1/P.m)*(sum_of_forces[3])
        x5ddot = (1/P.m)*(sum_of_forces[4])
        x6ddot = (1/P.m)*(sum_of_forces[5])
        x7ddot = (1/P.m)*(sum_of_forces[6])
        x8ddot = (1/P.m)*(sum_of_forces[7])
        x9ddot = (1/P.m)*(sum_of_forces[8])
        y1ddot = (1/P.m)*(sum_of_forces[9])
        y2ddot = (1/P.m)*(sum_of_forces[10])
        y3ddot = (1/P.m)*(sum_of_forces[11])
        y4ddot = (1/P.m)*(sum_of_forces[12])
        y5ddot = (1/P.m)*(sum_of_forces[13])
        y6ddot = (1/P.m)*(sum_of_forces[14])
        y7ddot = (1/P.m)*(sum_of_forces[15])
        y8ddot = (1/P.m)*(sum_of_forces[16])
        y9ddot = (1/P.m)*(sum_of_forces[17])
        z1ddot = (1/P.m)*(sum_of_forces[18])
        z2ddot = (1/P.m)*(sum_of_forces[19])
        z3ddot = (1/P.m)*(sum_of_forces[20])
        z4ddot = (1/P.m)*(sum_of_forces[21])
        z5ddot = (1/P.m)*(sum_of_forces[22])
        z6ddot = (1/P.m)*(sum_of_forces[23])
        z7ddot = (1/P.m)*(sum_of_forces[24])
        z8ddot = (1/P.m)*(sum_of_forces[25])
        z9ddot = (1/P.m)*(sum_of_forces[26])

        # Impose constraints on nodes 1, 2, and 3
        # Fill in the remaining values of xdot from the original state vector
        
        x1_dot = 0.0
        y1_dot = 0.0
        z1_dot = 0.0
        y2_dot = 0.0
        z2_dot = 0.0
        z3_dot = 0.0
        x1ddot = 0.0
        y1ddot = 0.0
        z1ddot = 0.0
        y2ddot = 0.0
        z2ddot = 0.0
        z3ddot = 0.0

        # Reconstruct the state vector
        xdot = np.array([[x1_dot], [y1_dot], [z1_dot], [x2_dot], [y2_dot], [z2_dot], [x3_dot], [y3_dot], [z3_dot],
                         [x4_dot], [y4_dot], [z4_dot], [x5_dot], [y5_dot], [z5_dot], [x6_dot], [y6_dot], [z6_dot],
                         [x7_dot], [y7_dot], [z7_dot], [x8_dot], [y8_dot], [z8_dot], [x9_dot], [y9_dot], [z9_dot],
                         [x1ddot], [y1ddot], [z1ddot], [x2ddot], [y2ddot], [z2ddot], [x3ddot], [y3ddot], [z3ddot],
                         [x4ddot], [y4ddot], [z4ddot], [x5ddot], [y5ddot], [z5ddot], [x6ddot], [y6ddot], [z6ddot],
                         [x7ddot], [y7ddot], [z7ddot], [x8ddot], [y8ddot], [z8ddot], [x9ddot], [y9ddot], [z9ddot]
                         ])
        return xdot
    
    def h(self):
        '''
        Function to collect and return the x, y, and z positions of the six nodes
        
        Returns:
            x: np.array of x positions of the six nodes (1, 2, 3, 4, 5, 6)
            y: np.array of y positions of the six nodes (1, 2, 3, 4, 5, 6)
            z: np.array of z positions of the six nodes (1, 2, 3, 4, 5, 6)
        '''
        x = np.zeros((9, 1))
        y = np.zeros((9, 1))
        z = np.zeros((9, 1))

        for i in range(9):
            x[i,0] = self.state[i*3][0]
            y[i,0] = self.state[i*3+1][0]
            z[i,0] = self.state[i*3+2][0]
        # x = np.array([[self.state[0][0]], [self.state[3][0]], [self.state[6][0]], [self.state[9][0]], [self.state[12][0]], [self.state[15][0]]])
        # y = np.array([[self.state[1][0]], [self.state[4][0]], [self.state[7][0]], [self.state[10][0]], [self.state[13][0]], [self.state[16][0]]])
        # z = np.array([[self.state[2][0]], [self.state[5][0]], [self.state[8][0]], [self.state[11][0]], [self.state[14][0]], [self.state[17][0]]])
        return x, y, z
    
    def rk4_step(self, tau):
        F1 = self.f(self.state, tau)
        F2 = self.f(self.state + self.Ts / 2.0 * F1, tau)
        F3 = self.f(self.state + self.Ts / 2.0 * F2, tau)
        F4 = self.f(self.state + self.Ts * F3, tau)
        self.state += self.Ts / 6.0 * (F1 + 2.0 * F2 + 2.0 * F3 + F4)

if __name__ == "__main__":
    import solar_array_sim



