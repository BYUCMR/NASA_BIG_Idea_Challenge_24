import numpy as np
from scipy.optimize import minimize
# from matlabengine import MatlabEngine
import time
import pickle
import csv



class Robot:
    

    def __init__(self, type):
        # eng = MatlabEngine()
        # Nodes = np.array([])
        # L_tube = 1
        # outputs = eng.eng.Generate_Shape_Order_Simple(Nodes, L_tube, nargout=5)
        # x_temp, Edges_Tube, b_t, n_true, tue_edge = outputs
        if type == 1:
            with open('GUI\octahedron.pkl', 'rb') as file:
                loaded_result = pickle.load(file)
                active_rollers = 8
        else:
            with open('GUI\solar.pkl', 'rb') as file:
                loaded_result = pickle.load(file)
                active_rollers = 14

        x_temp = loaded_result['x_temp']
        Edges_Tube = loaded_result['Edges_Tube']
        b_t = loaded_result['b_t']

        self.node_positions_along_tube = np.array([np.zeros(active_rollers)])

        self.pos = np.array(x_temp)
        self.edges = np.array(Edges_Tube).astype(np.int32)
        self.b_t = np.array(b_t)
        self.L2th = np.linalg.inv(self.b_t.T @ self.b_t) @ self.b_t.T
        self.rigidity_matrix_edges()
        self.get_loop_con()
        self.A_LoopCon = self.Loop_Con @ self.R
        self.b_LoopCon = np.zeros((self.Loop_Con.shape[0], 1))
        self.num_nodes = np.shape(self.pos)[0]

        self.support = np.array([0,1,2])


    def rigidity_matrix_edges(self):
        n = self.pos.shape[0]  # number of points
        d = self.pos.shape[1]  # dimensionality
        N_L = self.edges.shape[0]  # number of self.edges
        
        R = np.zeros((N_L, n * d))  # Initialize the rigidity matrix
        
        for i in range(N_L):
            # Extract the points
            x1 = self.pos[self.edges[i, 0], :]
            x2 = self.pos[self.edges[i, 1], :]
            norm_x1x2 = np.linalg.norm(x1 - x2)
            
            # Loop over the dimensions
            for j in range(d):
                R[i, self.edges[i, 0] + n * j] = (x1[j] - x2[j]) / norm_x1x2
                R[i, self.edges[i, 1] + n * j] = -R[i, self.edges[i, 0] + n * j]
        self.R = R
        return R
    

    def get_loop_con(self):
        triangles = len(self.edges)//3
        Loop_Con = np.array([])  # Initialize an empty array

        for _ in range(triangles):
            Elements = 3
            new_block = np.ones((1, Elements))  
            if Loop_Con.size == 0:  
                Loop_Con = new_block
            else:  
                Loop_Con = np.block([[Loop_Con, np.zeros((Loop_Con.shape[0], new_block.shape[1]))],
                                    [np.zeros((new_block.shape[0], Loop_Con.shape[1])), new_block]])
        self.Loop_Con = Loop_Con
        return Loop_Con
    
    def get_A_b_with_inputs(self, node_move, movement):
        
        n_all = np.shape(self.pos)[0]
        Node_Move = node_move
        Support = self.support

        b_con = np.array(movement)

        # Initialize C_Lock
        C_Lock = np.zeros((len(Support) * 3, 3 * n_all))
  
        # Assign values to C_Lock
        # Locks support[0] in all 3 axes
        C_Lock[0, Support[0]] = 1
        C_Lock[1, Support[0] + n_all] = 1
        C_Lock[2, Support[0] + 2 * n_all] = 1

        # Locks support[1] in y and z
        C_Lock[3, Support[1] + n_all] = 1
        C_Lock[4, Support[1] + 2 * n_all] = 1

        # Locks support[2] in z
        C_Lock[5, Support[2] + 2 * n_all] = 1

        # Lock the remaining nodes to fix all 3 support nodes
        C_Lock[6, Support[1]] = 1
        C_Lock[7, Support[2]] = 1
        C_Lock[8, Support[2] + n_all] = 1


        # Initialize C_Move
        C_Move = np.zeros((3, 3 * n_all))
        
        C_Move[0, Node_Move] = 1
        C_Move[1, Node_Move + n_all] = 1
        C_Move[2, Node_Move + 2 * n_all] = 1

        # Define matrices
        A_move = C_Move
        A_lock = C_Lock
        b_move = b_con
        b_lock = np.zeros((A_lock.shape[0], 1))

        # Combine matrices
        A = np.vstack((A_move, A_lock))
        b = np.vstack((b_move[:, np.newaxis], b_lock))
        self.A_con = A
        self.b_con = b
        return A,b

    def move(self, node, movement):
        self.rigidity_matrix_edges()
        self.A_LoopCon = self.Loop_Con @ self.R
        self.b_LoopCon = np.zeros((self.Loop_Con.shape[0], 1))
        Obj = np.array(self.R)
        H = 2 * (Obj.T @ Obj)
        f = np.zeros((np.shape(H)[0], 1))
        A,b = self.get_A_b_with_inputs(node, movement)
        Aeq=np.vstack((A, self.A_LoopCon))
        beq=np.vstack((b, self.b_LoopCon))

        # with open('output.csv', 'w', newline='') as file:
        #     writer = csv.writer(file)
        #     writer.writerows(Aeq)
        
        def objective(x, H, f):
            return 0.5 * np.dot(x.T, np.dot(H, x)) + np.dot(f.T, x)

        def constraint_eq(x, Aeq, beq):
            return np.dot(Aeq, x) - beq.flatten()
        
        x0 = np.zeros((np.shape(H)[0]))  # Initial guess

        # Define constraints for 'minimize'
        constraints = [
            {'type': 'eq', 'fun': constraint_eq, 'args': (Aeq, beq)},
        ]

        result = minimize(objective, x0, args=(H, f), method='trust-constr', constraints=constraints, options={'xtol': 1e-15, "disp": False})
        x_opt = result.x
        # TODO add check for if there is non 0 element in answer
        # print("\nAnswer\n")
        # print(Aeq @ np.array([x_opt]).T - beq)
        fval = result.fun
        exitflag = result.status
        output = result.message


        # with open('xopts.csv', 'a', newline='') as file:
        #     writer = csv.writer(file)
        #     writer.writerow(x_opt)

        x = self.pos[:,0]
        y = self.pos[:,1]
        z = self.pos[:,2]

        flattened_list = np.array([item for sublist in zip(*self.pos) for item in sublist])
        
        dt = 0.1
        real_world_const = 100
        new_pos = x_opt * dt + flattened_list
        self.l_dot = self.R @ x_opt
        command = real_world_const * self.L2th @ self.l_dot
        self.node_positions_along_tube = np.vstack([self.node_positions_along_tube, command * dt + self.node_positions_along_tube[-1, :]])
        pretty_str = np.array2string(self.node_positions_along_tube[-1,:], precision=2, separator=', ', suppress_small=True)
        print(pretty_str)
        unflattened_list = np.array([new_pos[i::self.num_nodes] for i in range(self.num_nodes)])
        self.pos = unflattened_list


    def plot_robot(self):
        x = self.pos[:, 0]
        y = self.pos[:, 1]
        z = self.pos[:, 2]
        return x, y, z, self.edges
                

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def plot_the_robot(robot, movement_node):
    
    x, y, z, edges = robot.plot_robot()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the points
    ax.scatter(x, y, z, color='b', s=50, label='Points')
    ax.scatter(x[movement_node], y[movement_node], z[movement_node], color='r', s=75, label='Points')

    # Plot the edges
    triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
    for idx, edge in enumerate(edges):
        p1, p2 = edge
        ax.plot([x[p1], x[p2]], [y[p1], y[p2]], [z[p1], z[p2]], triangle_colors[idx // 3], lw=1.5)

    # Add labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Add grid and set aspect ratio
    ax.grid(True)
    ax.set_box_aspect([1, 1, 1])  # This ensures equal aspect ratio

    # Add legend
    ax.legend()

    plt.show()

    



if __name__ == "__main__":

    r = Robot(2)
    movement_node = 5
    # for i in range(2):
    #     r.move(movement_node, [0,0,0.1])
    # print(r.L2th)
    plot_the_robot(r, movement_node)

