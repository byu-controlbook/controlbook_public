import numpy as np
import rodMassParam as P

class rodMassDynamics:
    def __init__(self, alpha=0.0):
        self.state = np.array([
            [0.0],
            [0.0]
        ])  # initial angular rate
        self.m = P.m * (1.+alpha*(2.*np.random.rand()-1.))  # Mass of the arm, kg
        self.ell = P.ell * (1.+alpha*(2.*np.random.rand()-1.))  # Length of the arm, m
        self.k1 = P.k1 * (1.+alpha*(2.*np.random.rand()-1.))  # Damping coefficient, Ns
        self.k2 = P.k2 * (1.+alpha*(2.*np.random.rand()-1.))  # Damping coefficient, Ns
        self.b = P.b * (1.+alpha*(2.*np.random.rand()-1.))  # Damping coefficient, Ns
        self.g = P.g  # the gravity constant is well known and so we don't change it.
        self.Ts = P.Ts  # sample rate at which the dynamics are propagated
        self.torque_limit = P.tau_max

    def update(self, u):
        u = self.saturate(u, self.torque_limit)
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, u):
        # Return xdot = f(x,u), the system state update equations
        # re-label states for readability
        theta = state[0][0]
        thetadot = state[1][0]
        tau = u
        xdot = np.array([
            [thetadot],
            [(-self.g/self.ell)*np.cos(theta)
             + (-self.k1/(self.m*self.ell**2))*theta
             + (-self.k2/(self.m*self.ell**2))*(theta**3)
             - self.b/(self.m*self.ell**2)*thetadot
             + (1/(self.m*self.ell**2))*tau],
        ])
        return xdot

    def h(self):
        # return the output equations
        # could also use input u if needed
        theta = self.state[0][0]
        y = theta
        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit * np.sign(u)
        return u