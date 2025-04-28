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
        self.num_nodes = 6  # Number of nodes in the octahedron

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
        node_positions = np.zeros((self.num_nodes, 3))
        node_velocities = np.zeros((self.num_nodes, 3))
        index_shift = self.num_nodes*3
        for i in range(self.num_nodes):
            node_positions[i] = np.array([state[3*i][0], state[3*i + 1][0], state[3*i + 2][0]])
            node_velocities[i] = np.array([state[3*i + index_shift][0], state[3*i + index_shift + 1][0], state[3*i + index_shift + 2][0]])

        self.RM.x = node_positions
        # Get the unit vectors of each side
        R = self.RM.Get_R()
        Edges = self.RM.Edges
        num_edges = int(np.size(Edges, 0))
        
        # Each row is one edge, every x component is grouped in the first six columns
        # Every y component is grouped in the next six columns
        # Every z component is grouped in the last six columns
        # Column 0 describes the x components of the unit vectors that end at node 1
        # Column 6 describes the y components of the unit vectors that end at node 1
        # Column 12 describes the z components of the unit vectors that end at node 1

        # Iteratively construct Fs for each tube (x12)
        Fs = np.zeros((num_edges))

        for i in range(num_edges):
            Fs[i] = -P.k*(np.linalg.norm(node_positions[Edges[i,0]] - node_positions[Edges[i,1]]) - self.mag[i])

        # Iteratively construct Fb for each tube (x12)
        Fb = np.zeros((num_edges))

        for i in range(num_edges):
            Fb[i] = -np.dot((node_positions[Edges[i,0]] - node_positions[Edges[i,1]]) / np.linalg.norm(node_positions[Edges[i,0]] - node_positions[Edges[i,1]]), node_velocities[Edges[i,0]] - node_velocities[Edges[i,1]])*P.b
        
        F = Fb + Fs
        gravity = P.g_vector    # List of the gravitational forces applied to each node in the -z directions

        sum_of_forces = F @ R + tau + gravity
        # Alternative equation can use R_T @ R
        
        # Compute accelerations from forces
        xdot = np.zeros((self.num_nodes*6, 1))

        for i in range(self.num_nodes):
            xdot[i*3 + index_shift, 0] = (1/P.m)*(sum_of_forces[i])
            xdot[i*3 + index_shift + 1, 0] = (1/P.m)*(sum_of_forces[i + self.num_nodes])
            xdot[i*3 + index_shift + 2, 0] = (1/P.m)*(sum_of_forces[i + 2*self.num_nodes])

        xdot[0:self.num_nodes*3, 0] = state[self.num_nodes*3:, 0]

        # Impose xyz constraints on nodes 1, 2, and 3
        ground_nodes = np.array([1, 2, 3]) - 1 # Subtract one to get the correct index
        constraints = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])
        xdot = self.constrain(xdot, ground_nodes, constraints)

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
    
    def constrain(self, xdot, nodes, constraints):
        '''
        Function to impose physical constraints on the dynamic system

        Inputs:
            xdot: np.array of the derivative of the state vector
            nodes: np.array of the node indices to be constrained
            constraints: np.array of the constraints to be imposed on the nodes
                Expects the order x, y, z for each node

        Returns:
            xdot: np.array of the derivative of the state vector with zeros entered for the constrained nodes
        '''

        index_shift = self.num_nodes*3

        for i in range(np.size(nodes)):
            for j in range(3):
                if constraints[i][j]:
                    xdot[nodes[i]*3 + j] = 0.0
                    xdot[nodes[i]*3 + j + index_shift] = 0.0
        return xdot

if __name__ == "__main__":
    import octahedron_1_sim



