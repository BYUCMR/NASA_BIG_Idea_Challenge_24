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
        num_nodes = np.size(x)/3

        self.state = np.zeros((num_states, 1)) # Initialize the state vector & fill with zeros
        for i in range(num_nodes): # fill the first num_nodes*3 indices with the x, y, z positions of the nodes
            # Note that the state vector is filled in multiples of 3 on each iteration
            self.state[3*i] = x[i, 0]
            self.state[3*i + 1] = x[i, 1]
            self.state[3*i + 2] = x[i, 2]

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
        node_positions = np.zeros((30, 3))
        node_velocities = np.zeros((30, 3))

        for i in range(30):
            node_positions[i] = np.array([state[3*i][0], state[3*i + 1][0], state[3*i + 2][0]])
            node_velocities[i] = np.array([state[3*i + 90][0], state[3*i + 91][0], state[3*i + 92][0]])

        # Additional variables for updating xdot at the end
        x1_dot = state[90][0]
        y1_dot = state[91][0]
        z1_dot = state[92][0]
        x2_dot = state[93][0]
        y2_dot = state[94][0]
        z2_dot = state[95][0]
        x3_dot = state[96][0]
        y3_dot = state[97][0]
        z3_dot = state[98][0]
        x4_dot = state[99][0]
        y4_dot = state[100][0]
        z4_dot = state[101][0]
        x5_dot = state[102][0]
        y5_dot = state[103][0]
        z5_dot = state[104][0]
        x6_dot = state[105][0]
        y6_dot = state[106][0]
        z6_dot = state[107][0]
        x7_dot = state[108][0]
        y7_dot = state[109][0]
        z7_dot = state[110][0]
        x8_dot = state[111][0]
        y8_dot = state[112][0]
        z8_dot = state[113][0]
        x9_dot = state[114][0]
        y9_dot = state[115][0]
        z9_dot = state[116][0]
        x10_dot = state[117][0]
        y10_dot = state[118][0]
        z10_dot = state[119][0]
        x11_dot = state[120][0]
        y11_dot = state[121][0]
        z11_dot = state[122][0]
        x12_dot = state[123][0]
        y12_dot = state[124][0]
        z12_dot = state[125][0]
        x13_dot = state[126][0]
        y13_dot = state[127][0]
        z13_dot = state[128][0]
        x14_dot = state[129][0]
        y14_dot = state[130][0]
        z14_dot = state[131][0]
        x15_dot = state[132][0]
        y15_dot = state[133][0]
        z15_dot = state[134][0]
        x16_dot = state[135][0]
        y16_dot = state[136][0]
        z16_dot = state[137][0]
        x17_dot = state[138][0]
        y17_dot = state[139][0]
        z17_dot = state[140][0]
        x18_dot = state[141][0]
        y18_dot = state[142][0]
        z18_dot = state[143][0]
        x19_dot = state[144][0]
        y19_dot = state[145][0]
        z19_dot = state[146][0]
        x20_dot = state[147][0]
        y20_dot = state[148][0]
        z20_dot = state[149][0]
        x21_dot = state[150][0]
        y21_dot = state[151][0]
        z21_dot = state[152][0]
        x22_dot = state[153][0]
        y22_dot = state[154][0]
        z22_dot = state[155][0]
        x23_dot = state[156][0]
        y23_dot = state[157][0]
        z23_dot = state[158][0]
        x24_dot = state[159][0]
        y24_dot = state[160][0]
        z24_dot = state[161][0]
        x25_dot = state[162][0]
        y25_dot = state[163][0]
        z25_dot = state[164][0]
        x26_dot = state[165][0]
        y26_dot = state[166][0]
        z26_dot = state[167][0]
        x27_dot = state[168][0]
        y27_dot = state[169][0]
        z27_dot = state[170][0]
        x28_dot = state[171][0]
        y28_dot = state[172][0]
        z28_dot = state[173][0]
        x29_dot = state[174][0]
        y29_dot = state[175][0]
        z29_dot = state[176][0]
        x30_dot = state[177][0]
        y30_dot = state[178][0]
        z30_dot = state[179][0]


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

        # Iteratively construct Fs for each tube (x84)
        Edges = self.RM.Edges
        
        Fs = np.zeros((84))
        
        for i in range(84):
            Fs[i] = -P.k*(np.linalg.norm(node_positions[Edges[i,0]] - node_positions[Edges[i,1]]) - mag[i])

        # Construct Fb for each tube (subtract the current node's velocity from the other node's velocity)
        Fb = np.zeros((84))

        for i in range(84):
            Fb[i] = -np.dot((node_positions[Edges[i,0]] - node_positions[Edges[i,1]]) / np.linalg.norm(node_positions[Edges[i,0]] - node_positions[Edges[i,1]]), node_velocities[Edges[i,0]] - node_velocities[Edges[i,1]])*P.b

        # Construct the force vector and add the external forces (90 item list of forces applied in the x/y/z directions to any of the nodes)
        # TODO: Include viscous friction against the ground for the bottom nodes to reduce rotary oscillations
        Ff = np.zeros((84))
        factor = 2.0
        # Ff[6] = node7_vel[0]*P.b*factor
        F = Fs + Fb #+ Ff
        gravity = P.g_vector    # List of the gravitational forces applied to each node in the -z directions

        sum_of_forces = F @ R + tau + gravity
        # Alternative equation can use R_T @ R
        
        # sum_of_forces will create a column vector of the sum of forces for nodes 1 through 30
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
        x10ddot = (1/P.m)*(sum_of_forces[9])
        x11ddot = (1/P.m)*(sum_of_forces[10])
        x12ddot = (1/P.m)*(sum_of_forces[11])
        x13ddot = (1/P.m)*(sum_of_forces[12])
        x14ddot = (1/P.m)*(sum_of_forces[13])
        x15ddot = (1/P.m)*(sum_of_forces[14])
        x16ddot = (1/P.m)*(sum_of_forces[15])
        x17ddot = (1/P.m)*(sum_of_forces[16])
        x18ddot = (1/P.m)*(sum_of_forces[17])
        x19ddot = (1/P.m)*(sum_of_forces[18])
        x20ddot = (1/P.m)*(sum_of_forces[19])
        x21ddot = (1/P.m)*(sum_of_forces[20])
        x22ddot = (1/P.m)*(sum_of_forces[21])
        x23ddot = (1/P.m)*(sum_of_forces[22])
        x24ddot = (1/P.m)*(sum_of_forces[23])
        x25ddot = (1/P.m)*(sum_of_forces[24])
        x26ddot = (1/P.m)*(sum_of_forces[25])
        x27ddot = (1/P.m)*(sum_of_forces[26])
        x28ddot = (1/P.m)*(sum_of_forces[27])
        x29ddot = (1/P.m)*(sum_of_forces[28])
        x30ddot = (1/P.m)*(sum_of_forces[29])
        y1ddot = (1/P.m)*(sum_of_forces[30])
        y2ddot = (1/P.m)*(sum_of_forces[31])
        y3ddot = (1/P.m)*(sum_of_forces[32])
        y4ddot = (1/P.m)*(sum_of_forces[33])
        y5ddot = (1/P.m)*(sum_of_forces[34])
        y6ddot = (1/P.m)*(sum_of_forces[35])
        y7ddot = (1/P.m)*(sum_of_forces[36])
        y8ddot = (1/P.m)*(sum_of_forces[37])
        y9ddot = (1/P.m)*(sum_of_forces[38])
        y10ddot = (1/P.m)*(sum_of_forces[39])
        y11ddot = (1/P.m)*(sum_of_forces[40])
        y12ddot = (1/P.m)*(sum_of_forces[41])
        y13ddot = (1/P.m)*(sum_of_forces[42])
        y14ddot = (1/P.m)*(sum_of_forces[43])
        y15ddot = (1/P.m)*(sum_of_forces[44])
        y16ddot = (1/P.m)*(sum_of_forces[45])
        y17ddot = (1/P.m)*(sum_of_forces[46])
        y18ddot = (1/P.m)*(sum_of_forces[47])
        y19ddot = (1/P.m)*(sum_of_forces[48])
        y20ddot = (1/P.m)*(sum_of_forces[49])
        y21ddot = (1/P.m)*(sum_of_forces[50])
        y22ddot = (1/P.m)*(sum_of_forces[51])
        y23ddot = (1/P.m)*(sum_of_forces[52])
        y24ddot = (1/P.m)*(sum_of_forces[53])
        y25ddot = (1/P.m)*(sum_of_forces[54])
        y26ddot = (1/P.m)*(sum_of_forces[55])
        y27ddot = (1/P.m)*(sum_of_forces[56])
        y28ddot = (1/P.m)*(sum_of_forces[57])
        y29ddot = (1/P.m)*(sum_of_forces[58])
        y30ddot = (1/P.m)*(sum_of_forces[59])
        z1ddot = (1/P.m)*(sum_of_forces[60])
        z2ddot = (1/P.m)*(sum_of_forces[61])
        z3ddot = (1/P.m)*(sum_of_forces[62])
        z4ddot = (1/P.m)*(sum_of_forces[63])
        z5ddot = (1/P.m)*(sum_of_forces[64])
        z6ddot = (1/P.m)*(sum_of_forces[65])
        z7ddot = (1/P.m)*(sum_of_forces[66])
        z8ddot = (1/P.m)*(sum_of_forces[67])
        z9ddot = (1/P.m)*(sum_of_forces[68])
        z10ddot = (1/P.m)*(sum_of_forces[69])
        z11ddot = (1/P.m)*(sum_of_forces[70])
        z12ddot = (1/P.m)*(sum_of_forces[71])
        z13ddot = (1/P.m)*(sum_of_forces[72])
        z14ddot = (1/P.m)*(sum_of_forces[73])
        z15ddot = (1/P.m)*(sum_of_forces[74])
        z16ddot = (1/P.m)*(sum_of_forces[75])
        z17ddot = (1/P.m)*(sum_of_forces[76])
        z18ddot = (1/P.m)*(sum_of_forces[77])
        z19ddot = (1/P.m)*(sum_of_forces[78])
        z20ddot = (1/P.m)*(sum_of_forces[79])
        z21ddot = (1/P.m)*(sum_of_forces[80])
        z22ddot = (1/P.m)*(sum_of_forces[81])
        z23ddot = (1/P.m)*(sum_of_forces[82])
        z24ddot = (1/P.m)*(sum_of_forces[83])
        z25ddot = (1/P.m)*(sum_of_forces[84])
        z26ddot = (1/P.m)*(sum_of_forces[85])
        z27ddot = (1/P.m)*(sum_of_forces[86])
        z28ddot = (1/P.m)*(sum_of_forces[87])
        z29ddot = (1/P.m)*(sum_of_forces[88])
        z30ddot = (1/P.m)*(sum_of_forces[89])


        # Impose constraints on nodes 20 (x, y, z), 24 (z), 25 (z), and 28 (z)
        # Fill in the remaining values of xdot from the original state vector
        x20_dot = 0.0
        y20_dot = 0.0
        z20_dot = 0.0
        x24_dot = 0.0
        y24_dot = 0.0
        z24_dot = 0.0
        x25_dot = 0.0
        y25_dot = 0.0
        z25_dot = 0.0
        x28_dot = 0.0
        y28_dot = 0.0
        z28_dot = 0.0

        x20ddot = 0.0
        y20ddot = 0.0
        z20ddot = 0.0
        x24ddot = 0.0
        y24ddot = 0.0
        z24ddot = 0.0
        x25ddot = 0.0
        y25ddot = 0.0
        z25ddot = 0.0
        x28ddot = 0.0
        y28ddot = 0.0
        z28ddot = 0.0

        # Reconstruct the state vector
        xdot = np.array([[x1_dot], [y1_dot], [z1_dot], [x2_dot], [y2_dot], [z2_dot], [x3_dot], [y3_dot], [z3_dot],
                         [x4_dot], [y4_dot], [z4_dot], [x5_dot], [y5_dot], [z5_dot], [x6_dot], [y6_dot], [z6_dot],
                         [x7_dot], [y7_dot], [z7_dot], [x8_dot], [y8_dot], [z8_dot], [x9_dot], [y9_dot], [z9_dot],
                         [x10_dot], [y10_dot], [z10_dot], [x11_dot], [y11_dot], [z11_dot], [x12_dot], [y12_dot], [z12_dot],
                         [x13_dot], [y13_dot], [z13_dot], [x14_dot], [y14_dot], [z14_dot], [x15_dot], [y15_dot], [z15_dot],
                         [x16_dot], [y16_dot], [z16_dot], [x17_dot], [y17_dot], [z17_dot], [x18_dot], [y18_dot], [z18_dot],
                         [x19_dot], [y19_dot], [z19_dot], [x20_dot], [y20_dot], [z20_dot], [x21_dot], [y21_dot], [z21_dot],
                         [x22_dot], [y22_dot], [z22_dot], [x23_dot], [y23_dot], [z23_dot], [x24_dot], [y24_dot], [z24_dot],
                         [x25_dot], [y25_dot], [z25_dot], [x26_dot], [y26_dot], [z26_dot], [x27_dot], [y27_dot], [z27_dot],
                         [x28_dot], [y28_dot], [z28_dot], [x29_dot], [y29_dot], [z29_dot], [x30_dot], [y30_dot], [z30_dot],
                         [x1ddot], [y1ddot], [z1ddot], [x2ddot], [y2ddot], [z2ddot], [x3ddot], [y3ddot], [z3ddot],
                         [x4ddot], [y4ddot], [z4ddot], [x5ddot], [y5ddot], [z5ddot], [x6ddot], [y6ddot], [z6ddot],
                         [x7ddot], [y7ddot], [z7ddot], [x8ddot], [y8ddot], [z8ddot], [x9ddot], [y9ddot], [z9ddot],
                         [x10ddot], [y10ddot], [z10ddot], [x11ddot], [y11ddot], [z11ddot], [x12ddot], [y12ddot], [z12ddot],
                         [x13ddot], [y13ddot], [z13ddot], [x14ddot], [y14ddot], [z14ddot], [x15ddot], [y15ddot], [z15ddot],
                         [x16ddot], [y16ddot], [z16ddot], [x17ddot], [y17ddot], [z17ddot], [x18ddot], [y18ddot], [z18ddot],
                         [x19ddot], [y19ddot], [z19ddot], [x20ddot], [y20ddot], [z20ddot], [x21ddot], [y21ddot], [z21ddot],
                         [x22ddot], [y22ddot], [z22ddot], [x23ddot], [y23ddot], [z23ddot], [x24ddot], [y24ddot], [z24ddot],
                         [x25ddot], [y25ddot], [z25ddot], [x26ddot], [y26ddot], [z26ddot], [x27ddot], [y27ddot], [z27ddot],
                         [x28ddot], [y28ddot], [z28ddot], [x29ddot], [y29ddot], [z29ddot], [x30ddot], [y30ddot], [z30ddot]
                         ])
        return xdot
    
    def h(self):
        '''
        Function to collect and return the x, y, and z positions of the thirty nodes
        
        Returns:
            x: np.array of x positions of the six nodes (1, 2, 3, 4, 5, 6, ... 30)
            y: np.array of y positions of the six nodes (1, 2, 3, 4, 5, 6, ... 30)
            z: np.array of z positions of the six nodes (1, 2, 3, 4, 5, 6, ... 30)
        '''
        x = np.zeros((30, 1))
        y = np.zeros((30, 1))
        z = np.zeros((30, 1))

        for i in range(30):
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
    import crane_sim


    '''
        xdot = np.zeros((180, 1))

        # Set the xddot, yddot, and zddot values based on the sum of forces equations
        for i in range(30):
            xdot[90 + i*3, 0] = (1/P.m)*(sum_of_forces[i*3])
            xdot[i*3 + 91, 0] = (1/P.m)*(sum_of_forces[i*3 + 1])
            xdot[i*3 + 92, 0] = (1/P.m)*(sum_of_forces[i*3 + 2])

        # Fill in the remaining values of xdot from the velocity values of the original state vector
        xdot[0:90, 0] = state[90:180, 0]

        # Impose constraints on nodes 20 (x, y, z), 24 (z), 25 (z), and 28 (z)
        # Fill in the remaining values of xdot from the original state vector
        xdot[57] = 0.0
        xdot[58] = 0.0
        xdot[59] = 0.0
        xdot[71] = 0.0
        xdot[74] = 0.0
        xdot[83] = 0.0
        xdot[147] = 0.0
        xdot[148] = 0.0
        xdot[149] = 0.0
        xdot[161] = 0.0
        xdot[164] = 0.0
        xdot[173] = 0.0
    '''