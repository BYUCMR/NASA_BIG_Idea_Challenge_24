# This code outlines the dynamics of the original Nasa Truss dynamic model

import numpy as np
import nodeParam as P

class NodeDynamics:
    def __init__(self):
        # Initial state conditions
        self.state = np.array([[P.x1_dot], [P.x1], [P.y1_dot], [P.y1]])
        
        self.Ts = P.Ts

    def update(self, u):
        self.rk4_step(u)
        x = self.h()

    def f(self, state, tau):
        x1_dot = state[0][0]
        x1 = state[1][0]
        y1_dot = state[2][0]
        y1 = state[3][0]

        # The equations of motion.
        x1ddot = (1/P.m)*(P.m*P.g*np.cos(P.theta)+tau*np.cos(P.theta)-P.k*x1-P.b*x1_dot)
        y1ddot = (1/P.m)*(P.m*P.g*np.sin(P.theta)+tau*np.sin(P.theta)-P.k*y1-P.b*y1_dot)

        xdot = np.array([[x1ddot], [x1_dot], [y1ddot], [y1_dot]])

        return xdot
    
    def h(self):
        x1 = self.state[1][0]
        y1 = self.state[3][0]
        return np.array([[x1], [y1]])
    
    def rk4_step(self, tau):
        F1 = self.f(self.state, tau)
        F2 = self.f(self.state + self.Ts / 2.0 * F1, tau)
        F3 = self.f(self.state + self.Ts / 2.0 * F2, tau)
        F4 = self.f(self.state + self.Ts * F3, tau)
        self.state += self.Ts / 6.0 * (F1 + 2.0*F2 + 2.0*F3 + F4)


if __name__ == "__main__":
    # run the model_1_sim file
    import model_1_sim