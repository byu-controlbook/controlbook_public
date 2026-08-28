import numpy as np 
import VTOLParam as P


class VTOLDynamics:
    def __init__(self, alpha = 0.0):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # initial lateral position
            [P.h0],  # initial altitude
            [P.theta0],  # initial roll angle
            [P.zdot0],  # initial lateral velocity
            [P.hdot0],  # initial climb rate
            [P.thetadot0],  # initial angular velocity
        ])
        #################################################
        # The parameters for any physical system are never known exactly. Feedback
        # systems need to be designed to be robust to this uncertainty. In the simulation
        # we model uncertainty by changing the physical parameters by a uniform random variable
        # that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
        # may change by up to 20%. A different parameter value is chosen every time the simulation
        # is run. This solution does not require the "alpha" parameter to be defined unless we want
        # to model uncertainty in our model. This is something that comes later in the book when
        # doing feedback control.
        #################################################
        self.mc = P.mc * (1+2*alpha*np.random.rand()-alpha)
        self.mr = P.mr * (1+2*alpha*np.random.rand()-alpha)
        self.Jc = P.Jc * (1+2*alpha*np.random.rand()-alpha)
        self.d = P.d * (1+2*alpha*np.random.rand()-alpha)
        self.mu = P.mu * (1+2*alpha*np.random.rand()-alpha)
        self.F_wind = P.F_wind * (1+2*alpha*np.random.rand()-alpha)

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        self.rk4_step(u)  # propagate the state by one time sample
        # separating out "y" by itself is currently not required, but will be in future homework
        y = self.h()  # using a measurement model, return the corresponding output
        return y

    def f(self, state, u):
        #  Return xdot = f(x,u)
        z = state[0, 0]
        h = state[1, 0]
        theta = state[2, 0]
        zdot = state[3, 0]
        hdot = state[4, 0]
        thetadot = state[5, 0]
        fr = u[0, 0]
        fl = u[1, 0]
        # The equations of motion.
        zddot = (-(fr + fl) * np.sin(theta) + -self.mu * zdot + self.F_wind) / (self.mc + 2.0 * self.mr)
        hddot = (-(self.mc + 2.0 * self.mr) * P.g + (fr + fl) * np.cos(theta)) / (self.mc + 2.0 * self.mr)
        thetaddot = self.d * (fr - fl) / (self.Jc + 2.0 * self.mr * (self.d ** 2))
        # build xdot and return
        xdot = np.array([[zdot], [hdot], [thetadot], [zddot], [hddot], [thetaddot]])
        return xdot

    def h(self):
        # return y=h(x)
        z = self.state.item(0)
        h = self.state.item(1)
        theta = self.state.item(2)
        y = np.array([[z], [h], [theta]])
        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + P.Ts / 2 * F1, u)
        F3 = self.f(self.state + P.Ts / 2 * F2, u)
        F4 = self.f(self.state + P.Ts * F3, u)
        self.state = self.state + self.Ts / 6 * (F1 + 2*F2 + 2*F3 + F4)

