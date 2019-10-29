import numpy as np
import pendulumParam as P

class pendulumDynamics:
    def __init__(self):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # initial position
            [P.theta0],  # initial orientation
            [P.zdot0],  # initial velocity
            [P.thetadot0], # initial angular velocity
        ])
        self.Ts = P.Ts
        self.m1 = P.m1  # Mass of the pendulum, kg
        self.m2 = P.m2  # Mass of the cart, kg
        self.ell = P.ell  # Length of the rod, m
        self.b = P.b  # Damping coefficient, Ns
        self.g = P.g  # gravity

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def rk1_step(self, u):
        # Integrate ODE using Runge-Kutta RK1 algorithm
        self.state += self.Ts * self.f(self.state, u)

    def rk2_step(self, u):
        # Integrate ODE using Runge-Kutta RK2 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts * F1, u)
        self.state += self.Ts / 2 * (F1 + F2)

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)


    def f(self, state, u):
        '''
            Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
        '''
        # re-label states and inputs for readability
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)
        F = u
        # The equations of motion.
        M = np.matrix([[self.m1+self.m2, self.m1*(self.ell/2.0)*np.cos(theta)],
                       [self.m1*(self.ell/2.0)*np.cos(theta), self.m1*(self.ell**2/3.0)]])
        C = np.matrix([[self.m1*(self.ell/2.0)*thetadot**2*np.sin(theta) + F - self.b*zdot],
                       [self.m1*self.g*(self.ell/2.0)*np.sin(theta)]])
        tmp = np.linalg.inv(M)*C
        zddot = tmp.item(0)
        thetaddot = tmp.item(1)
        # build xdot and return
        xdot = np.array([[zdot], [thetadot], [zddot], [thetaddot]])
        return xdot

    def h(self):
        z = self.state.item(0)
        theta = self.state.item(1)
        y = np.array([
            [z],
            [theta],
        ])
        return y