import numpy as np
import param as P

class dynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = 0
        self.Ts = P.Ts

    def update(self, u):
        self.rk4_step(u)
        y = self.h()
        return y

    def f(self, state, u):
        # Return xdot = f(x,u), the system state update equations
        xdot = -state + u
        return xdot

    def h(self):
        y = self.state
        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)