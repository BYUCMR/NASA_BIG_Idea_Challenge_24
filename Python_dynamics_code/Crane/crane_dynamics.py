import numpy as np
import crane_param as P
from crane_rigidity_matrix import CraneRigidityMatrix

class CraneDynamics:
    """
    This class outlines the dynamics of a crane truss structure
    """
    # TODO: Add the ability to change the lengths of the tubes according to a kinematic solution
    # Needs to be implemented within the 'f' function, likely through the RM object

    def __init__(self):
        # Initial state condition. Requires x, y, z, and xdot, ydot, and zdot for each of the nodes
        # Order of the states: x, y, z for all nodes, followed by xdot, ydot, zdot for all nodes
        
        self.RM = CraneRigidityMatrix()
        x = self.RM.x
        num_states = np.size(x)*2
        self.num_nodes = int(np.size(x)/3)

        self.state = np.zeros((num_states, 1)) # Initialize the state vector & fill with zeros
        for i in range(self.num_nodes): # fill the first num_nodes*3 indices with the x, y, z positions of the nodes
            # Note that the state vector is filled in multiples of 3 on each iteration
            self.state[3*i, 0] = x[i, 0]
            self.state[3*i + 1, 0] = x[i, 1]
            self.state[3*i + 2, 0] = x[i, 2]

        # Leave the second half of the state vector as zeros for the velocities (initialized when self.state was created)
        
        self.Ts = P.Ts

    def update(self, u):
        self.rk4_step(u)
        x = self.h()

    def f(self, state, tau = np.zeros((90))):
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
        # Create a position and velocity vector of all the nodes
        node_positions = np.zeros((self.num_nodes, 3))
        node_velocities = np.zeros((self.num_nodes, 3))
        index_shift = self.num_nodes*3
        for i in range(self.num_nodes):
            node_positions[i] = np.array([state[3*i][0], state[3*i + 1][0], state[3*i + 2][0]])
            node_velocities[i] = np.array([state[3*i + index_shift][0], state[3*i + index_shift + 1][0], state[3*i + index_shift + 2][0]])

        # TODO: Update the rigidity matrix based on the kinematics files (matlab or python)
        # Get the magnitudes of each side length
        mag = self.RM.Get_Lengths()
        # Get the unit vectors of each side
        R = self.RM.Get_R()
        # Each row is one edge, every x component is grouped in the first thirty columns
        # Every y component is grouped in the next thirty columns
        # Every z component is grouped in the last thirty columns
        # Column 0 describes the x components of the unit vectors that end at node 1
        # Column 30 describes the y components of the unit vectors that end at node 1
        # Column 60 describes the z components of the unit vectors that end at node 1

        # Iteratively construct Fs for each tube (stored in num_edges)
        Edges = self.RM.Edges
        num_edges = int(np.size(Edges, 0))

        Fs = np.zeros((num_edges))
        
        for i in range(num_edges):
            Fs[i] = -P.k*(np.linalg.norm(node_positions[Edges[i,0]] - node_positions[Edges[i,1]]) - mag[i])

        # Construct Fb for each tube (subtract the current node's velocity from the other node's velocity)
        Fb = np.zeros((num_edges))

        for i in range(num_edges):
            Fb[i] = -np.dot((node_positions[Edges[i,0]] - node_positions[Edges[i,1]]) / np.linalg.norm(node_positions[Edges[i,0]] - node_positions[Edges[i,1]]), node_velocities[Edges[i,0]] - node_velocities[Edges[i,1]])*P.b

        # TODO: Include viscous friction against the ground for the bottom nodes to reduce rotary oscillations
        # Ff = np.zeros((self.num_nodes*3))
        # ground_nodes = np.array([20, 24, 25, 28]) - 1 # Subtract one to get the correct index
        # factor = 3.0
        # for i in ground_nodes:
        #     Ff[i*3] = -node_velocities[i][0]*P.b*factor
        #     Ff[i*3 + 1] = -node_velocities[i][1]*P.b*factor

        # Construct the force vector and add the external forces (num_nodes*3 item list of forces applied in the x/y/z directions to each of the nodes)
        F = Fs + Fb
        gravity = P.g_vector    # List of the gravitational forces applied to each node in the -z directions
        sum_of_forces = F @ R + tau + gravity #+ Ff
        
        # sum_of_forces will create a column vector of the sum of forces for nodes 1 through 30
        # in the x direction, followed by the y and z directions
        # Reconstruct the state vector using the equations of motion
        xdot = np.zeros((self.num_nodes*6, 1))

        # Set the xddot, yddot, and zddot values based on the sum of forces equations
        for i in range(self.num_nodes):
            xdot[i*3 + index_shift, 0] = (1/P.m)*(sum_of_forces[i])
            xdot[i*3 + index_shift + 1, 0] = (1/P.m)*(sum_of_forces[i + self.num_nodes])
            xdot[i*3 + index_shift + 2, 0] = (1/P.m)*(sum_of_forces[i + 2*self.num_nodes])
        
        xdot[0:self.num_nodes*3, 0] = state[self.num_nodes*3:, 0]

        # Impose constraints on nodes 20 (x, y, z), 24 (z), 25 (z), and 28 (z)
        ground_nodes = np.array([20, 24, 25, 28]) - 1 # Subtract one to get the correct index
        # Constraints are in the order x, y, z, with 1 indicating the constraint is active
        constraints = np.array([[1, 1, 1], [1, 1, 1], [0, 1, 1], [0, 1, 1]])

        xdot = self.constrain(xdot, ground_nodes, constraints)
        
        return xdot
    
    def h(self):
        '''
        Function to collect and return the x, y, and z positions of the nodes
        
        Returns:
            x: np.array of x positions of the six nodes (1, 2, 3... num_nodes)
            y: np.array of y positions of the six nodes (1, 2, 3... num_nodes)
            z: np.array of z positions of the six nodes (1, 2, 3... num_nodes)
        '''
        x = np.zeros((self.num_nodes, 1))
        y = np.zeros((self.num_nodes, 1))
        z = np.zeros((self.num_nodes, 1))

        for i in range(self.num_nodes):
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
    import crane_sim