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
        self.state = np.array([[P.x1],[P.y1],[P.z1],[P.x2],[P.y2],[P.z2],[P.x3],[P.y3],[P.z3],
                               [P.x4],[P.y4],[P.z4],[P.x5],[P.y5],[P.z5],[P.x6],[P.y6],[P.z6],
                               [P.x7],[P.y7],[P.z7],[P.x8],[P.y8],[P.z8],[P.x9],[P.y9],[P.z9],
                               [P.x10],[P.y10],[P.z10],[P.x11],[P.y11],[P.z11],[P.x12],[P.y12],[P.z12],
                               [P.x13],[P.y13],[P.z13],[P.x14],[P.y14],[P.z14],[P.x15],[P.y15],[P.z15],
                               [P.x16],[P.y16],[P.z16],[P.x17],[P.y17],[P.z17],[P.x18],[P.y18],[P.z18],
                               [P.x19],[P.y19],[P.z19],[P.x20],[P.y20],[P.z20],[P.x21],[P.y21],[P.z21],
                               [P.x22],[P.y22],[P.z22],[P.x23],[P.y23],[P.z23],[P.x24],[P.y24],[P.z24],
                               [P.x25],[P.y25],[P.z25],[P.x26],[P.y26],[P.z26],[P.x27],[P.y27],[P.z27],
                               [P.x28],[P.y28],[P.z28],[P.x29],[P.y29],[P.z29],[P.x30],[P.y30],[P.z30],
                               [P.x1_dot],[P.y1_dot],[P.z1_dot],[P.x2_dot],[P.y2_dot],[P.z2_dot],[P.x3_dot],[P.y3_dot],[P.z3_dot],
                               [P.x4_dot],[P.y4_dot],[P.z4_dot],[P.x5_dot],[P.y5_dot],[P.z5_dot],[P.x6_dot],[P.y6_dot],[P.z6_dot],
                               [P.x7_dot],[P.y7_dot],[P.z7_dot],[P.x8_dot],[P.y8_dot],[P.z8_dot],[P.x9_dot],[P.y9_dot],[P.z9_dot],
                               [P.x10_dot],[P.y10_dot],[P.z10_dot],[P.x11_dot],[P.y11_dot],[P.z11_dot],[P.x12_dot],[P.y12_dot],[P.z12_dot],
                               [P.x13_dot],[P.y13_dot],[P.z13_dot],[P.x14_dot],[P.y14_dot],[P.z14_dot],[P.x15_dot],[P.y15_dot],[P.z15_dot],
                               [P.x16_dot],[P.y16_dot],[P.z16_dot],[P.x17_dot],[P.y17_dot],[P.z17_dot],[P.x18_dot],[P.y18_dot],[P.z18_dot],
                               [P.x19_dot],[P.y19_dot],[P.z19_dot],[P.x20_dot],[P.y20_dot],[P.z20_dot],[P.x21_dot],[P.y21_dot],[P.z21_dot],
                               [P.x22_dot],[P.y22_dot],[P.z22_dot],[P.x23_dot],[P.y23_dot],[P.z23_dot],[P.x24_dot],[P.y24_dot],[P.z24_dot],
                               [P.x25_dot],[P.y25_dot],[P.z25_dot],[P.x26_dot],[P.y26_dot],[P.z26_dot],[P.x27_dot],[P.y27_dot],[P.z27_dot],
                               [P.x28_dot],[P.y28_dot],[P.z28_dot],[P.x29_dot],[P.y29_dot],[P.z29_dot],[P.x30_dot],[P.y30_dot],[P.z30_dot]])
        
        self.RM = CraneRigidityMatrix()
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
        node10_pos = np.array([state[27][0], state[28][0], state[29][0]])
        node11_pos = np.array([state[30][0], state[31][0], state[32][0]])
        node12_pos = np.array([state[33][0], state[34][0], state[35][0]])
        node13_pos = np.array([state[36][0], state[37][0], state[38][0]])
        node14_pos = np.array([state[39][0], state[40][0], state[41][0]])
        node15_pos = np.array([state[42][0], state[43][0], state[44][0]])
        node16_pos = np.array([state[45][0], state[46][0], state[47][0]])
        node17_pos = np.array([state[48][0], state[49][0], state[50][0]])
        node18_pos = np.array([state[51][0], state[52][0], state[53][0]])
        node19_pos = np.array([state[54][0], state[55][0], state[56][0]])
        node20_pos = np.array([state[57][0], state[58][0], state[59][0]])
        node21_pos = np.array([state[60][0], state[61][0], state[62][0]])
        node22_pos = np.array([state[63][0], state[64][0], state[65][0]])
        node23_pos = np.array([state[66][0], state[67][0], state[68][0]])
        node24_pos = np.array([state[69][0], state[70][0], state[71][0]])
        node25_pos = np.array([state[72][0], state[73][0], state[74][0]])
        node26_pos = np.array([state[75][0], state[76][0], state[77][0]])
        node27_pos = np.array([state[78][0], state[79][0], state[80][0]])
        node28_pos = np.array([state[81][0], state[82][0], state[83][0]])
        node29_pos = np.array([state[84][0], state[85][0], state[86][0]])
        node30_pos = np.array([state[87][0], state[88][0], state[89][0]])

        node1_vel = np.array([state[90][0], state[91][0], state[92][0]])
        node2_vel = np.array([state[93][0], state[94][0], state[95][0]])
        node3_vel = np.array([state[96][0], state[97][0], state[98][0]])
        node4_vel = np.array([state[99][0], state[100][0], state[101][0]])
        node5_vel = np.array([state[102][0], state[103][0], state[104][0]])
        node6_vel = np.array([state[105][0], state[106][0], state[107][0]])
        node7_vel = np.array([state[108][0], state[109][0], state[110][0]])
        node8_vel = np.array([state[111][0], state[112][0], state[113][0]])
        node9_vel = np.array([state[114][0], state[115][0], state[116][0]])
        node10_vel = np.array([state[117][0], state[118][0], state[119][0]])
        node11_vel = np.array([state[120][0], state[121][0], state[122][0]])
        node12_vel = np.array([state[123][0], state[124][0], state[125][0]])
        node13_vel = np.array([state[126][0], state[127][0], state[128][0]])
        node14_vel = np.array([state[129][0], state[130][0], state[131][0]])
        node15_vel = np.array([state[132][0], state[133][0], state[134][0]])
        node16_vel = np.array([state[135][0], state[136][0], state[137][0]])
        node17_vel = np.array([state[138][0], state[139][0], state[140][0]])
        node18_vel = np.array([state[141][0], state[142][0], state[143][0]])
        node19_vel = np.array([state[144][0], state[145][0], state[146][0]])
        node20_vel = np.array([state[147][0], state[148][0], state[149][0]])
        node21_vel = np.array([state[150][0], state[151][0], state[152][0]])
        node22_vel = np.array([state[153][0], state[154][0], state[155][0]])
        node23_vel = np.array([state[156][0], state[157][0], state[158][0]])
        node24_vel = np.array([state[159][0], state[160][0], state[161][0]])
        node25_vel = np.array([state[162][0], state[163][0], state[164][0]])
        node26_vel = np.array([state[165][0], state[166][0], state[167][0]])
        node27_vel = np.array([state[168][0], state[169][0], state[170][0]])
        node28_vel = np.array([state[171][0], state[172][0], state[173][0]])
        node29_vel = np.array([state[174][0], state[175][0], state[176][0]])
        node30_vel = np.array([state[177][0], state[178][0], state[179][0]])
        

        # TODO: Create code to identify which nodes are connected via tubes and use that to iteratively
        # construct the Fs and Fb vectors
        node_positions = np.array([node1_pos, node2_pos, node3_pos, node4_pos, node5_pos, node6_pos, node7_pos, node8_pos, node9_pos, node10_pos, node11_pos, node12_pos, node13_pos, node14_pos, node15_pos, node16_pos, node17_pos, node18_pos, node19_pos, node20_pos, node21_pos, node22_pos, node23_pos, node24_pos, node25_pos, node26_pos, node27_pos, node28_pos, node29_pos, node30_pos])
        node_velocities = np.array([node1_vel, node2_vel, node3_vel, node4_vel, node5_vel, node6_vel, node7_vel, node8_vel, node9_vel, node10_vel, node11_vel, node12_vel, node13_vel, node14_vel, node15_vel, node16_vel, node17_vel, node18_vel, node19_vel, node20_vel, node21_vel, node22_vel, node23_vel, node24_vel, node25_vel, node26_vel, node27_vel, node28_vel, node29_vel, node30_vel])

        # Additional variables for updating xdot at the end
        x1_dot = state[90][0]
        y1_dot = state[91][0]
        z1_dot = state[92][0]
        x2_dot = state[93][0]
        y2_dot = state[94][0]
        z2_dot = state[95][0]
        x4_dot = state[99][0]
        y4_dot = state[100][0]
        z4_dot = state[101][0]
        x5_dot = state[102][0]
        y5_dot = state[103][0]
        z5_dot = state[104][0]
        x6_dot = state[105][0]
        y6_dot = state[106][0]
        x7_dot = state[108][0]
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
        Fs = np.zeros((84))
        Fs[0] = -P.k*(np.linalg.norm(node2_pos - node1_pos) - mag[0])
        Fs[1] = -P.k*(np.linalg.norm(node3_pos - node2_pos) - mag[1])
        Fs[2] = -P.k*(np.linalg.norm(node1_pos - node3_pos) - mag[2])
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
        Fs[13] = -P.k*(np.linalg.norm(node3_pos - node7_pos) - mag[13])
        Fs[14] = -P.k*(np.linalg.norm(node8_pos - node3_pos) - mag[14])
        Fs[15] = -P.k*(np.linalg.norm(node5_pos - node8_pos) - mag[15])
        Fs[16] = -P.k*(np.linalg.norm(node9_pos - node5_pos) - mag[16])
        Fs[17] = -P.k*(np.linalg.norm(node8_pos - node9_pos) - mag[17])
        Fs[18] = -P.k*(np.linalg.norm(node7_pos - node4_pos) - mag[18])
        Fs[19] = -P.k*(np.linalg.norm(node9_pos - node7_pos) - mag[19])
        Fs[20] = -P.k*(np.linalg.norm(node4_pos - node9_pos) - mag[20])
        Fs[21] = -P.k*(np.linalg.norm(node10_pos - node11_pos) - mag[21])
        Fs[22] = -P.k*(np.linalg.norm(node2_pos - node10_pos) - mag[22])
        Fs[23] = -P.k*(np.linalg.norm(node11_pos - node2_pos) - mag[23])
        Fs[24] = -P.k*(np.linalg.norm(node3_pos - node11_pos) - mag[24])
        Fs[25] = -P.k*(np.linalg.norm(node12_pos - node3_pos) - mag[25])
        Fs[26] = -P.k*(np.linalg.norm(node11_pos - node12_pos) - mag[26])
        Fs[27] = -P.k*(np.linalg.norm(node10_pos - node1_pos) - mag[27])
        Fs[28] = -P.k*(np.linalg.norm(node12_pos - node10_pos) - mag[28])
        Fs[29] = -P.k*(np.linalg.norm(node1_pos - node12_pos) - mag[29])
        Fs[30] = -P.k*(np.linalg.norm(node13_pos - node14_pos) - mag[30])
        Fs[31] = -P.k*(np.linalg.norm(node2_pos - node13_pos) - mag[31])
        Fs[32] = -P.k*(np.linalg.norm(node14_pos - node2_pos) - mag[32])
        Fs[33] = -P.k*(np.linalg.norm(node5_pos - node14_pos) - mag[33])
        Fs[34] = -P.k*(np.linalg.norm(node15_pos - node5_pos) - mag[34])
        Fs[35] = -P.k*(np.linalg.norm(node14_pos - node15_pos) - mag[35])
        Fs[36] = -P.k*(np.linalg.norm(node13_pos - node3_pos) - mag[36])
        Fs[37] = -P.k*(np.linalg.norm(node15_pos - node13_pos) - mag[37])
        Fs[38] = -P.k*(np.linalg.norm(node3_pos - node15_pos) - mag[38])
        Fs[39] = -P.k*(np.linalg.norm(node16_pos - node17_pos) - mag[39])
        Fs[40] = -P.k*(np.linalg.norm(node3_pos - node16_pos) - mag[40])
        Fs[41] = -P.k*(np.linalg.norm(node17_pos - node3_pos) - mag[41])
        Fs[42] = -P.k*(np.linalg.norm(node4_pos - node17_pos) - mag[42])
        Fs[43] = -P.k*(np.linalg.norm(node18_pos - node4_pos) - mag[43])
        Fs[44] = -P.k*(np.linalg.norm(node17_pos - node18_pos) - mag[44])
        Fs[45] = -P.k*(np.linalg.norm(node16_pos - node1_pos) - mag[45])
        Fs[46] = -P.k*(np.linalg.norm(node18_pos - node16_pos) - mag[46])
        Fs[47] = -P.k*(np.linalg.norm(node1_pos - node18_pos) - mag[47])
        Fs[48] = -P.k*(np.linalg.norm(node19_pos - node20_pos) - mag[48])
        Fs[49] = -P.k*(np.linalg.norm(node11_pos - node19_pos) - mag[49])
        Fs[50] = -P.k*(np.linalg.norm(node20_pos - node11_pos) - mag[50])
        Fs[51] = -P.k*(np.linalg.norm(node12_pos - node20_pos) - mag[51])
        Fs[52] = -P.k*(np.linalg.norm(node21_pos - node12_pos) - mag[52])
        Fs[53] = -P.k*(np.linalg.norm(node20_pos - node21_pos) - mag[53])
        Fs[54] = -P.k*(np.linalg.norm(node19_pos - node10_pos) - mag[54])
        Fs[55] = -P.k*(np.linalg.norm(node21_pos - node19_pos) - mag[55])
        Fs[56] = -P.k*(np.linalg.norm(node10_pos - node21_pos) - mag[56])
        Fs[57] = -P.k*(np.linalg.norm(node22_pos - node23_pos) - mag[57])
        Fs[58] = -P.k*(np.linalg.norm(node14_pos - node22_pos) - mag[58])
        Fs[59] = -P.k*(np.linalg.norm(node23_pos - node14_pos) - mag[59])
        Fs[60] = -P.k*(np.linalg.norm(node15_pos - node23_pos) - mag[60])
        Fs[61] = -P.k*(np.linalg.norm(node24_pos - node15_pos) - mag[61])
        Fs[62] = -P.k*(np.linalg.norm(node23_pos - node24_pos) - mag[62])
        Fs[63] = -P.k*(np.linalg.norm(node22_pos - node13_pos) - mag[63])
        Fs[64] = -P.k*(np.linalg.norm(node24_pos - node22_pos) - mag[64])
        Fs[65] = -P.k*(np.linalg.norm(node13_pos - node24_pos) - mag[65])
        Fs[66] = -P.k*(np.linalg.norm(node25_pos - node26_pos) - mag[66])
        Fs[67] = -P.k*(np.linalg.norm(node8_pos - node25_pos) - mag[67])
        Fs[68] = -P.k*(np.linalg.norm(node26_pos - node8_pos) - mag[68])
        Fs[69] = -P.k*(np.linalg.norm(node9_pos - node26_pos) - mag[69])
        Fs[70] = -P.k*(np.linalg.norm(node27_pos - node9_pos) - mag[70])
        Fs[71] = -P.k*(np.linalg.norm(node26_pos - node27_pos) - mag[71])
        Fs[72] = -P.k*(np.linalg.norm(node25_pos - node7_pos) - mag[72])
        Fs[73] = -P.k*(np.linalg.norm(node27_pos - node25_pos) - mag[73])
        Fs[74] = -P.k*(np.linalg.norm(node7_pos - node27_pos) - mag[74])
        Fs[75] = -P.k*(np.linalg.norm(node28_pos - node29_pos) - mag[75])
        Fs[76] = -P.k*(np.linalg.norm(node17_pos - node28_pos) - mag[76])
        Fs[77] = -P.k*(np.linalg.norm(node29_pos - node17_pos) - mag[77])
        Fs[78] = -P.k*(np.linalg.norm(node18_pos - node29_pos) - mag[78])
        Fs[79] = -P.k*(np.linalg.norm(node30_pos - node18_pos) - mag[79])
        Fs[80] = -P.k*(np.linalg.norm(node29_pos - node30_pos) - mag[80])
        Fs[81] = -P.k*(np.linalg.norm(node28_pos - node16_pos) - mag[81])
        Fs[82] = -P.k*(np.linalg.norm(node30_pos - node28_pos) - mag[82])
        Fs[83] = -P.k*(np.linalg.norm(node16_pos - node30_pos) - mag[83])




        # Construct Fb for each tube (subtract the current node's velocity from the other node's velocity)
        Fb = np.zeros((84))
        Fb[0] = -np.dot((node2_pos - node1_pos) / np.linalg.norm(node2_pos - node1_pos), node2_vel - node1_vel)*P.b
        Fb[1] = -np.dot((node3_pos - node2_pos) / np.linalg.norm(node3_pos - node2_pos), node3_vel - node2_vel)*P.b
        Fb[2] = -np.dot((node1_pos - node3_pos) / np.linalg.norm(node1_pos - node3_pos), node1_vel - node3_vel)*P.b
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
        Fb[13] = -np.dot((node3_pos - node7_pos) / np.linalg.norm(node3_pos - node7_pos), node3_vel - node7_vel)*P.b
        Fb[14] = -np.dot((node8_pos - node3_pos) / np.linalg.norm(node8_pos - node3_pos), node8_vel - node3_vel)*P.b
        Fb[15] = -np.dot((node5_pos - node8_pos) / np.linalg.norm(node5_pos - node8_pos), node5_vel - node8_vel)*P.b
        Fb[16] = -np.dot((node9_pos - node5_pos) / np.linalg.norm(node9_pos - node5_pos), node9_vel - node5_vel)*P.b
        Fb[17] = -np.dot((node8_pos - node9_pos) / np.linalg.norm(node8_pos - node9_pos), node8_vel - node9_vel)*P.b
        Fb[18] = -np.dot((node7_pos - node4_pos) / np.linalg.norm(node7_pos - node4_pos), node7_vel - node4_vel)*P.b
        Fb[19] = -np.dot((node9_pos - node7_pos) / np.linalg.norm(node9_pos - node7_pos), node9_vel - node7_vel)*P.b
        Fb[20] = -np.dot((node4_pos - node9_pos) / np.linalg.norm(node4_pos - node9_pos), node4_vel - node9_vel)*P.b
        Fb[21] = -np.dot((node10_pos - node11_pos) / np.linalg.norm(node10_pos - node11_pos), node10_vel - node11_vel)*P.b
        Fb[22] = -np.dot((node2_pos - node10_pos) / np.linalg.norm(node2_pos - node10_pos), node2_vel - node10_vel)*P.b
        Fb[23] = -np.dot((node11_pos - node2_pos) / np.linalg.norm(node11_pos - node2_pos), node11_vel - node2_vel)*P.b
        Fb[24] = -np.dot((node3_pos - node11_pos) / np.linalg.norm(node3_pos - node11_pos), node3_vel - node11_vel)*P.b
        Fb[25] = -np.dot((node12_pos - node3_pos) / np.linalg.norm(node12_pos - node3_pos), node12_vel - node3_vel)*P.b
        Fb[26] = -np.dot((node11_pos - node12_pos) / np.linalg.norm(node11_pos - node12_pos), node11_vel - node12_vel)*P.b
        Fb[27] = -np.dot((node10_pos - node1_pos) / np.linalg.norm(node10_pos - node1_pos), node10_vel - node1_vel)*P.b
        Fb[28] = -np.dot((node12_pos - node10_pos) / np.linalg.norm(node12_pos - node10_pos), node12_vel - node10_vel)*P.b
        Fb[29] = -np.dot((node1_pos - node12_pos) / np.linalg.norm(node1_pos - node12_pos), node1_vel - node12_vel)*P.b
        Fb[30] = -np.dot((node13_pos - node14_pos) / np.linalg.norm(node13_pos - node14_pos), node13_vel - node14_vel)*P.b
        Fb[31] = -np.dot((node2_pos - node13_pos) / np.linalg.norm(node2_pos - node13_pos), node2_vel - node13_vel)*P.b
        Fb[32] = -np.dot((node14_pos - node2_pos) / np.linalg.norm(node14_pos - node2_pos), node14_vel - node2_vel)*P.b
        Fb[33] = -np.dot((node5_pos - node14_pos) / np.linalg.norm(node5_pos - node14_pos), node5_vel - node14_vel)*P.b
        Fb[34] = -np.dot((node15_pos - node5_pos) / np.linalg.norm(node15_pos - node5_pos), node15_vel - node5_vel)*P.b
        Fb[35] = -np.dot((node14_pos - node15_pos) / np.linalg.norm(node14_pos - node15_pos), node14_vel - node15_vel)*P.b
        Fb[36] = -np.dot((node13_pos - node3_pos) / np.linalg.norm(node13_pos - node3_pos), node13_vel - node3_vel)*P.b
        Fb[37] = -np.dot((node15_pos - node13_pos) / np.linalg.norm(node15_pos - node13_pos), node15_vel - node13_vel)*P.b
        Fb[38] = -np.dot((node3_pos - node15_pos) / np.linalg.norm(node3_pos - node15_pos), node3_vel - node15_vel)*P.b
        Fb[39] = -np.dot((node16_pos - node17_pos) / np.linalg.norm(node16_pos - node17_pos), node16_vel - node17_vel)*P.b
        Fb[40] = -np.dot((node3_pos - node16_pos) / np.linalg.norm(node3_pos - node16_pos), node3_vel - node16_vel)*P.b
        Fb[41] = -np.dot((node17_pos - node3_pos) / np.linalg.norm(node17_pos - node3_pos), node17_vel - node3_vel)*P.b
        Fb[42] = -np.dot((node4_pos - node17_pos) / np.linalg.norm(node4_pos - node17_pos), node4_vel - node17_vel)*P.b
        Fb[43] = -np.dot((node18_pos - node4_pos) / np.linalg.norm(node18_pos - node4_pos), node18_vel - node4_vel)*P.b
        Fb[44] = -np.dot((node17_pos - node18_pos) / np.linalg.norm(node17_pos - node18_pos), node17_vel - node18_vel)*P.b
        Fb[45] = -np.dot((node16_pos - node1_pos) / np.linalg.norm(node16_pos - node1_pos), node16_vel - node1_vel)*P.b
        Fb[46] = -np.dot((node18_pos - node16_pos) / np.linalg.norm(node18_pos - node16_pos), node18_vel - node16_vel)*P.b
        Fb[47] = -np.dot((node1_pos - node18_pos) / np.linalg.norm(node1_pos - node18_pos), node1_vel - node18_vel)*P.b
        Fb[48] = -np.dot((node19_pos - node20_pos) / np.linalg.norm(node19_pos - node20_pos), node19_vel - node20_vel)*P.b
        Fb[49] = -np.dot((node11_pos - node19_pos) / np.linalg.norm(node11_pos - node19_pos), node11_vel - node19_vel)*P.b
        Fb[50] = -np.dot((node20_pos - node11_pos) / np.linalg.norm(node20_pos - node11_pos), node20_vel - node11_vel)*P.b
        Fb[51] = -np.dot((node12_pos - node20_pos) / np.linalg.norm(node12_pos - node20_pos), node12_vel - node20_vel)*P.b
        Fb[52] = -np.dot((node21_pos - node12_pos) / np.linalg.norm(node21_pos - node12_pos), node21_vel - node12_vel)*P.b
        Fb[53] = -np.dot((node20_pos - node21_pos) / np.linalg.norm(node20_pos - node21_pos), node20_vel - node21_vel)*P.b
        Fb[54] = -np.dot((node19_pos - node10_pos) / np.linalg.norm(node19_pos - node10_pos), node19_vel - node10_vel)*P.b
        Fb[55] = -np.dot((node21_pos - node19_pos) / np.linalg.norm(node21_pos - node19_pos), node21_vel - node19_vel)*P.b
        Fb[56] = -np.dot((node10_pos - node21_pos) / np.linalg.norm(node10_pos - node21_pos), node10_vel - node21_vel)*P.b
        Fb[57] = -np.dot((node22_pos - node23_pos) / np.linalg.norm(node22_pos - node23_pos), node22_vel - node23_vel)*P.b
        Fb[58] = -np.dot((node14_pos - node22_pos) / np.linalg.norm(node14_pos - node22_pos), node14_vel - node22_vel)*P.b
        Fb[59] = -np.dot((node23_pos - node14_pos) / np.linalg.norm(node23_pos - node14_pos), node23_vel - node14_vel)*P.b
        Fb[60] = -np.dot((node15_pos - node23_pos) / np.linalg.norm(node15_pos - node23_pos), node15_vel - node23_vel)*P.b
        Fb[61] = -np.dot((node24_pos - node15_pos) / np.linalg.norm(node24_pos - node15_pos), node24_vel - node15_vel)*P.b
        Fb[62] = -np.dot((node23_pos - node24_pos) / np.linalg.norm(node23_pos - node24_pos), node23_vel - node24_vel)*P.b
        Fb[63] = -np.dot((node22_pos - node13_pos) / np.linalg.norm(node22_pos - node13_pos), node22_vel - node13_vel)*P.b
        Fb[64] = -np.dot((node24_pos - node22_pos) / np.linalg.norm(node24_pos - node22_pos), node24_vel - node22_vel)*P.b
        Fb[65] = -np.dot((node13_pos - node24_pos) / np.linalg.norm(node13_pos - node24_pos), node13_vel - node24_vel)*P.b
        Fb[66] = -np.dot((node25_pos - node26_pos) / np.linalg.norm(node25_pos - node26_pos), node25_vel - node26_vel)*P.b
        Fb[67] = -np.dot((node8_pos - node25_pos) / np.linalg.norm(node8_pos - node25_pos), node8_vel - node25_vel)*P.b
        Fb[68] = -np.dot((node26_pos - node8_pos) / np.linalg.norm(node26_pos - node8_pos), node26_vel - node8_vel)*P.b
        Fb[69] = -np.dot((node9_pos - node26_pos) / np.linalg.norm(node9_pos - node26_pos), node9_vel - node26_vel)*P.b
        Fb[70] = -np.dot((node27_pos - node9_pos) / np.linalg.norm(node27_pos - node9_pos), node27_vel - node9_vel)*P.b
        Fb[71] = -np.dot((node26_pos - node27_pos) / np.linalg.norm(node26_pos - node27_pos), node26_vel - node27_vel)*P.b
        Fb[72] = -np.dot((node25_pos - node7_pos) / np.linalg.norm(node25_pos - node7_pos), node25_vel - node7_vel)*P.b
        Fb[73] = -np.dot((node27_pos - node25_pos) / np.linalg.norm(node27_pos - node25_pos), node27_vel - node25_vel)*P.b
        Fb[74] = -np.dot((node7_pos - node27_pos) / np.linalg.norm(node7_pos - node27_pos), node7_vel - node27_vel)*P.b
        Fb[75] = -np.dot((node28_pos - node29_pos) / np.linalg.norm(node28_pos - node29_pos), node28_vel - node29_vel)*P.b
        Fb[76] = -np.dot((node17_pos - node28_pos) / np.linalg.norm(node17_pos - node28_pos), node17_vel - node28_vel)*P.b
        Fb[77] = -np.dot((node29_pos - node17_pos) / np.linalg.norm(node29_pos - node17_pos), node29_vel - node17_vel)*P.b
        Fb[78] = -np.dot((node18_pos - node29_pos) / np.linalg.norm(node18_pos - node29_pos), node18_vel - node29_vel)*P.b
        Fb[79] = -np.dot((node30_pos - node18_pos) / np.linalg.norm(node30_pos - node18_pos), node30_vel - node18_vel)*P.b
        Fb[80] = -np.dot((node29_pos - node30_pos) / np.linalg.norm(node29_pos - node30_pos), node29_vel - node30_vel)*P.b
        Fb[81] = -np.dot((node28_pos - node16_pos) / np.linalg.norm(node28_pos - node16_pos), node28_vel - node16_vel)*P.b
        Fb[82] = -np.dot((node30_pos - node28_pos) / np.linalg.norm(node30_pos - node28_pos), node30_vel - node28_vel)*P.b
        Fb[83] = -np.dot((node16_pos - node30_pos) / np.linalg.norm(node16_pos - node30_pos), node16_vel - node30_vel)*P.b


        # Construct the force vector and add the external forces (90 item list of forces applied in the x/y/z directions to any of the nodes)
        # TODO: Include viscous friction against the ground for the bottom nodes to reduce rotary oscillations
        Ff = np.zeros((84))
        factor = 2.0
        Ff[6] = node7_vel[0]*P.b*factor
        F = Fs + Fb + Ff
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


        # Impose constraints on nodes 3 (x, y, z), 6 (z), 7 (y, z), and 11 (z)
        # Fill in the remaining values of xdot from the original state vector
        
        x3_dot = 0.0
        y3_dot = 0.0
        z3_dot = 0.0
        z6_dot = 0.0
        y7_dot = 0.0
        z7_dot = 0.0
        z11_dot = 0.0
        x3ddot = 0.0
        y3ddot = 0.0
        z3ddot = 0.0
        z6ddot = 0.0
        y7ddot = 0.0
        z7ddot = 0.0
        z11ddot = 0.0

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



