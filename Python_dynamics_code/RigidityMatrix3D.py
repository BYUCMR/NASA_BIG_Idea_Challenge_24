"""
Class and functions to compute edge lengths, and rigidity matrix for octohedron (3D)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RigidityMatrix3D:
    def __init__(self, plot=False):

        x = np.array([[0.5, 0, -0.5, 0, -0.5, 0.5]])
        y = np.array([[-0.2887, 0.5774, -0.2887, -0.5774, 0.2887, 0.2887]])
        z = np.array([[0, 0, 0, 0.8165, 0.8165, 0.8165]])
        
        self.x = np.hstack([np.transpose(x), np.transpose(y), np.transpose(z)])

        if plot:
            fig = plt.figure()
            ax = plt.axes(projection='3d')
            plt.plot(x, y, z, '+-')
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            plt.show()

        # Update to pull from matlab functions in the future
        self.Edges = np.array([
            [0, 1], [2, 1], [0, 2], 
            [4, 3], [3, 2], [2, 4], 
            [4, 1], [1, 5], [5, 4], 
            [0, 3], [3, 5], [5, 0]
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
    
    def Get_Lengths(self):
        m = self.Edges.shape[0]
        Length = np.zeros(m)
        for i in range(0,m):
            Length[i] = np.linalg.norm(self.x[self.Edges[i,0],:] - self.x[self.Edges[i,1],:])
        return Length
    
    def Get_R(self):
        m = self.Edges.shape[0] # Find the number of edges
        d = self.x.shape[1] # The dimension of the space - this should make it work for any number of dimensions
        n = self.x.shape[0] # Fine the number of nodes
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
    RM = RigidityMatrix3D(True)
    print(RM.x)
    A = RM.Get_A_from_Edges()
    print(A)
    Length = RM.Get_Lengths()
    print(Length)
    R = RM.Get_R()
    print(R)


