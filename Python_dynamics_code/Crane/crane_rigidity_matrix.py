"""
Class and functions to compute edge lengths, and rigidity matrix for the 3D model of the crane.

NOTE: To produce the rigidity matrix for a new model, all that is required is the definition of the x, y, and z coordinates of each node,
as well as their respective connections (edges). The rest of the class is generic and will work for any model.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class CraneRigidityMatrix:
    def __init__(self, plot=False):
        
        x = np.array([[0.7872, -0.7872, 0.0, 0.7872, -0.7872, 0.0, 0.7872, -0.7872, 0.0, 0.0, -0.7872, 0.7872, -1.3121, -2.0993, -1.3121, 1.3121, 1.3121, 2.0993, -0.7872, 0.0, 0.7872, -2.8865, -2.8865, -2.0993, 0.0, -0.7872, 0.7872, 2.0993, 2.8865, 2.8865]])
        y = np.array([[0.7872, 0.7872, 0.0, -0.7872, -0.7872, 0.0, -1.3121, -1.3121, -2.0993, 2.0993, 1.3121, 1.3121, 0.7872, 0.0, -0.7872, 0.7872, -0.7872, 0.0, 2.8865, 2.0993, 2.8865, 0.7872, -0.7872, 0.0, -2.0993, -2.8865, -2.8865, 0.0, -0.7872, 0.7872]])
        z = np.array([[2.5977, 2.5977, 1.4844, 2.5977, 2.5977, 3.7111, 1.1133, 1.1133, 2.2266, 2.2266, 1.1133, 1.1133, 1.1133, 2.2266, 1.1133, 1.1133, 1.1133, 2.2266, 1.1133, 0.0, 1.1133, 1.1133, 1.1133, 0.0, 0.0, 1.1133, 1.1133, 0.0, 1.1133, 1.1133]])
        
        self.x = np.hstack([np.transpose(x), np.transpose(y), np.transpose(z)])

        if plot:
            fig = plt.figure()
            ax = plt.axes(projection='3d')
            plt.plot(x.T, y.T, z.T, '+-')
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            plt.show()

        # Update to pull from matlab functions in the future
        self.Edges = np.array([
            [0, 1], [1, 2], [2, 0], [4, 3], [3, 2], [2, 4], 
            [4, 1], [1, 5], [5, 4], [0, 3], [3, 5], [5, 0],
            [7, 6], [6, 2], [2, 7], [7, 4], [4, 8], [8, 7],
            [3, 6], [6, 8], [8, 3], [10, 9], [9, 1], [1, 10],
            [10, 2], [2, 11], [11, 10], [0, 9], [9, 11], [11, 0],
            [13, 12], [12, 1], [1, 13], [13, 4], [4, 14], [14, 13],
            [12, 14], [14, 2], [16, 15], [15, 2], [2, 16],
            [16, 3], [3, 17], [17, 16], [0, 15], [15, 17], [17, 0],
            [19, 18], [18, 10], [10, 19], [19, 11], [11, 20], [20, 19],
            [9, 18], [18, 20], [20, 9], [22, 21], [21, 13], [13, 22],
            [22, 14], [14, 23], [23, 22], [12, 21], [21, 23], [23, 12],
            [25, 24], [24, 7], [7, 25], [25, 8], [8, 26], [26, 25],
            [6, 24], [24, 26], [26, 6], [28, 27], [27, 16], [16, 28],
            [28, 17], [17, 29], [29, 28], [15, 27], [27, 29], [29, 15]
        ])
        # NOTE: I cut this out because it doesn't appear to be a valid connection: [3, 13]

    def Get_A_from_Edges(self):
        # Get the adjacency matrix from the list of edges
        m = self.Edges.shape[0]
        n = self.x.shape[0]
        A = np.zeros((n, n))
        # Loop over each edge, inserting one connection
        for i in range(0,m):
            A[self.Edges[i,0], self.Edges[i,1]] = 1

        A = A + A.T
        return A
    
    def Get_Lengths(self):
        m = self.Edges.shape[0]
        Length = np.zeros(m)
        for i in range(0,m):
            Length[i] = np.linalg.norm(self.x[self.Edges[i,0],:] - self.x[self.Edges[i,1],:])
        return Length
    
    def Get_R(self):
        m = self.Edges.shape[0] # Find the number of edges
        d = self.x.shape[1] # The dimension of the space - this should make it work for any number of dimensions
        n = self.x.shape[0] # Find the number of nodes
        R = np.zeros((m, n*d)) # Initialize the rigidity matrix
        for i in range(0,m):
            x1 = self.x[self.Edges[i,0],:]
            x2 = self.x[self.Edges[i,1],:]
            normx1x2 = np.linalg.norm(x1-x2)
            for j in range(0,d):
                R[i, self.Edges[i,0] + n*j] = (x1[j] - x2[j]) / normx1x2
                R[i, self.Edges[i,1] + n*j] = -(x1[j] - x2[j]) / normx1x2
        return R
    
if __name__ == "__main__":
    # Test the class
    RM = CraneRigidityMatrix(True)
    print(RM.x)
    A = RM.Get_A_from_Edges()
    print(A)
    Length = RM.Get_Lengths()
    print(Length)
    R = RM.Get_R()
    print(R)


