import numpy as np
import triangleParam2D as P
from RigidityMatrix2D import RigidityMatrix2D

class TriangleDynamics:
    """
    This class outlines the dynamics of a single triangle truss structure
    """
    # TODO: Add the ability to change the lengths of the tubes according to a kinematic solution
    # Needs to be implemented within the 'f' function, likely through the RM object
    def __init__(self):
        # initial state conditions. Requires x, y, z, and xdot, ydot, and zdot for each of the 12 nodes
        # Order of the states: x, y, z for all nodes, followed by xdot, ydot, zdot for all nodes
        self.state = np.array([[P.x1],[P.y1],[P.x2],[P.y2],[P.x3],[P.y3],
                               [P.x1_dot],[P.y1_dot],[P.x2_dot],[P.y2_dot],[P.x3_dot],[P.y3_dot]])
        self.RM = RigidityMatrix2D()
        self.Ts = P.Ts

    def update(self, u):
        self.rk4_step(u)
        x = self.h()
    
    def f(self, state, tau=np.zeros((6))):
        '''
        Function to calculate the derivative of the state vector

        Inputs:
            state: np.array of the current state vector
            tau: np.array of the external forces applied to the nodes
                Expects the order of x, y, for node 1, x, y for node 2, etc.
                Then the xdot, ydot for node 1, etc.

        Returns:
            xdot: np.array of the derivative of the state vector
        '''
        node1_pos = np.array([state[0][0], state[1][0]])
        node2_pos = np.array([state[2][0], state[3][0]])
        node3_pos = np.array([state[4][0], state[5][0]])

        node1_vel = np.array([state[6][0], state[7][0]])
        node2_vel = np.array([state[8][0], state[9][0]])
        node3_vel = np.array([state[10][0], state[11][0]])

        # Additional variables for updating xdot at the end
        # x1_dot = state[6][0]  # Set to zero later as part of the constraints
        # y1_dot = state[7][0]  # Set to zero later as part of the constraints
        x2_dot = state[8][0]
        # y2_dot = state[9][0]  # Set to zero later as part of the constraints
        x3_dot = state[10][0]
        y3_dot = state[11][0]

        # Get the magnitudes of each side length
        mag = self.RM.Get_Lengths()
        # Get the unit vectors of each side
        R = self.RM.Get_R()
        # Each row is one edge, every x component is grouped in the first three columns
        # Every y component is grouped in the last three columns
        # Column 0 describes the x components of the unit vectors that end at node 1
        # Column 3 describes the y components of the unit vectors that end at node 1
        
        #Iteratively construct Fs for each tube (x3)
        Fs = np.zeros((3))
        Fs[0] = -P.k*(np.linalg.norm(node2_pos-node1_pos)-mag[0])
        Fs[1] = -P.k*(np.linalg.norm(node3_pos-node2_pos)-mag[1])
        Fs[2] = -P.k*(np.linalg.norm(node1_pos-node3_pos)-mag[2])

        # Construct Fb for each tube (subtract the current node's velocity from the other node's velocity)
        Fb = np.zeros((3))
        Fb[0] = -np.dot((node2_pos - node1_pos) / np.linalg.norm(node2_pos - node1_pos), node2_vel - node1_vel)*P.b
        Fb[1] = -np.dot((node3_pos - node2_pos) / np.linalg.norm(node3_pos - node2_pos), node3_vel - node2_vel)*P.b
        Fb[2] = -np.dot((node1_pos - node3_pos) / np.linalg.norm(node1_pos - node3_pos), node1_vel - node3_vel)*P.b
        
        # Ff = np.zeros((3))
        # Ff[1] = -np.dot(np.array([1, 0]), node2_vel)*P.b*0.0
        # Construct the force vector and add the external forces (6x1 column vector of forces applied in the x/y directions to nodes 1, 2, and/or 3)
        F = Fs + Fb # + Ff
        sum_of_forces = F @ R + tau + P.g_vector
        # sum of forces will create a column vector of the sum of forces for nodes 1, 2, and 3, in the x direction, followed by the y direction

        # Reconstruct the state vector using the equations of motion
        x1ddot = (1/P.m)*(sum_of_forces[0])
        x2ddot = (1/P.m)*(sum_of_forces[1])
        x3ddot = (1/P.m)*(sum_of_forces[2])
        y1ddot = (1/P.m)*(sum_of_forces[3])
        y2ddot = (1/P.m)*(sum_of_forces[4])
        y3ddot = (1/P.m)*(sum_of_forces[5])
            
        # Impose a constraint on node 1 of zero x and y motion, and on node 2 of zero y motion
        x1_dot = 0.0
        y1_dot = 0.0
        y2_dot = 0.0
        x1ddot = 0.0
        y1ddot = 0.0
        y2ddot = 0.0

        # reconstruct the state vector
        xdot = np.array([[x1_dot], [y1_dot], [x2_dot], [y2_dot], [x3_dot], [y3_dot], [x1ddot], [y1ddot], [x2ddot], [y2ddot], [x3ddot], [y3ddot]])
        return xdot

    def h(self):
        '''
        Function to collect and return the x and y positions of the three nodes

        Returns:
        x: np.array of x positions of the three nodes (1, 2, 3)
        y: np.array of y positions of the three nodes (1, 2, 3)
        '''
        x = np.array([[self.state[0][0]], [self.state[2][0]], [self.state[4][0]]])
        y = np.array([[self.state[1][0]], [self.state[3][0]], [self.state[5][0]]])
        return x, y

    def rk4_step(self, tau):
        F1 = self.f(self.state, tau)
        F2 = self.f(self.state + self.Ts / 2.0 * F1, tau)
        F3 = self.f(self.state + self.Ts / 2.0 * F2, tau)
        F4 = self.f(self.state + self.Ts * F3, tau)
        self.state += self.Ts / 6.0 * (F1 + 2.0*F2 + 2.0*F3 + F4)

if __name__ == "__main__":
    import Python_dynamics_code.Triangle.triangle_1_sim as triangle_1_sim