import param as P
import numpy as np

class Dynamics:
    def __init__(self, alpha=0.0):
        self.state = np.array([[0.0], [0.0]])
        self.A = P.A * (1.+alpha*(2.*np.random.rand()-1.))
        self.B = P.B * (1.+alpha*(2.*np.random.rand()-1.))
        self.C = P.C * (1.+alpha*(2.*np.random.rand()-1.))
        self.D = P.D * (1.+alpha*(2.*np.random.rand()-1.))
        self.Ts = P.Ts

    def update(self, u):
        self.rk4_step(u)  
        y = self.h()  
        return y

    def f(self, state, u):
        xdot = self.A @ state + self.B * u
        return xdot

    def h(self):
        y = self.C @ self.state
        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2.0 * F1, u)
        F3 = self.f(self.state + self.Ts / 2.0 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6.0 * (F1 + 2.0 * F2 + 2.0 * F3 + F4)


