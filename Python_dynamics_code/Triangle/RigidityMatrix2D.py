# -*- coding: utf-8 -*-
"""
Class and functions to compute edge lengths, and rigidity matrix for simple 2D case
"""

import numpy as np
import matplotlib.pyplot as plt

class RigidityMatrix2D:
    def __init__(self, plot=False):
        L1 = 1
        L2 = 1
        L3 = 1
        th = np.arccos((L1**2 + L2**2 - L3**2)/(2*L1*L2))

        x1=np.array([[0, L2, L1*np.cos(th)]])
        y=np.array([[0, 0,  L1*np.sin(th)]])

        self.x=np.hstack([np.transpose(x1), np.transpose(y)])

        # Possibly include a boolean later to determine if the plot is shown
        if plot:
            plt.plot(x1, y,'+-')
            plt.axis('equal')
            plt.show()

        # Update to pull from matlab functions in the future
        self.Edges=np.array([[0, 1],[1, 2],[2, 0]])

    def Get_A_from_Edges(self):
        #Get the adjacency matrix from the list of the edges
        m= self.Edges.shape[0]
        n= self.x.shape[0]
        A=np.zeros((n,n))   
        #Loop over each edge, inserting one connection
        for i in range(0,m):
            A[self.Edges[i,0], self.Edges[i,1]]= 1
        
        A=A+A.transpose() #Make sure adjacency is symetric
        return A

    def Get_Lengths(self):
        m= self.Edges.shape[0]
        Length= np.zeros(m)
        for i in range(0, m):
            Length[i]= np. linalg. norm( self.x[self.Edges[i,0],:] - self.x[self.Edges[i,1],:]) 
        return Length

    def Get_R(self):
        m= self.Edges.shape[0] # Find the number of edges
        d= self.x.shape[1] #The dimension of the points.  This should make it work for 2D or 3D
        n=self.x.shape[0]
        R=np.zeros((m, n*d))
        for i in range(0, m):
            x1=self.x[self.Edges[i,0],:]
            x2=self.x[self.Edges[i,1],:]
            normx1x2= np.linalg.norm(x1-x2)
            for j in range(0, d):
                R[i, self.Edges[i,0]  + n * j]= (x1[j]- x2[j]) / normx1x2
                R[i, self.Edges[i,1]  + n * j]= -(x1[j]- x2[j]) / normx1x2
        return R
    

if __name__ == "__main__":
    # Test to see if the Rigidity Matrix Class correctly returns each of the expected results
    RM = RigidityMatrix2D(True)
    print(RM.x)
    A = RM.Get_A_from_Edges()
    print(A)
    Length = RM.Get_Lengths()
    print(Length)
    R = RM.Get_R()
    print(R)