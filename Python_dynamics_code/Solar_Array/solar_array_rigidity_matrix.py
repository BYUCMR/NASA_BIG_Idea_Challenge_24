"""
Class and functions to compute edge lengths, and rigidity matrix for the 3D model of the solar panel array

NOTE: To produce the rigidity matrix for a new model, all that is required is the definition of the x, y, and z coordinates of each node,
as well as their respective connections (edges). The rest of the class is generic and will work for any model.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class SolarRigidityMatrix:
    def __init__(self, plot=False, x=None):
        
        x1 = np.array([[0.787, 0.0, -0.787, 0.0, -0.787, 0.787, 0.0, 0.787, -0.787]])
        y = np.array([[-0.45, 0.909, -0.45, -0.909, 0.45, 0.45, 0.909, -0.45, -0.45]])
        z = np.array([[0.0, 0.0, 0.0, 1.286, 1.286, 1.286, 2.571, 2.571, 2.571]])
        
        self.x = np.hstack([np.transpose(x1), np.transpose(y), np.transpose(z)])

        if x is None:
            x = self.x

        if plot:
            fig = plt.figure()
            ax = plt.axes(projection='3d')
            plt.plot(x[:,0], x[:,1], x[:,2], '+-')
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            plt.show()

        # Update to pull from matlab functions in the future
        self.Edges = np.array([
            [0, 1], [1, 2], [2, 0], 
            [4, 3], [3, 2], [2, 4], 
            [4, 1], [1, 5], [5, 4], 
            [0, 3], [3, 5], [5, 0],
            [7, 6], [6, 5], [5, 7],
            [7, 3], [3, 8], [8, 7],
            [4, 6], [6, 8], [8, 4]
        ])

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
    
    def Get_Lengths(self, x=None):
        if x is None:
            x = self.x
        m = self.Edges.shape[0]
        Length = np.zeros(m)
        for i in range(0,m):
            Length[i] = np.linalg.norm(x[self.Edges[i,0],:] - x[self.Edges[i,1],:])
        return Length
    
    def Get_R(self,x=None):
        if x is None:
            x = self.x
        m = self.Edges.shape[0] # Find the number of edges
        d = self.x.shape[1] # The dimension of the space - this should make it work for any number of dimensions
        n = self.x.shape[0] # Fine the number of nodes
        R = np.zeros((m, n*d)) # Initialize the rigidity matrix
        for i in range(0,m):
            x1 = x[self.Edges[i,0],:]
            x2 = x[self.Edges[i,1],:]
            normx1x2 = np.linalg.norm(x1-x2)
            for j in range(0,d):
                R[i, self.Edges[i,0] + n*j] = (x1[j] - x2[j]) / normx1x2
                R[i, self.Edges[i,1] + n*j] = -(x1[j] - x2[j]) / normx1x2
        return R
    
if __name__ == "__main__":
    # Test the class
    RM = SolarRigidityMatrix(True)
    print(RM.x)
    A = RM.Get_A_from_Edges()
    print(A)
    Length = RM.Get_Lengths()
    print(Length)
    R = RM.Get_R()
    print(R)


