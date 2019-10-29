import numpy as np 
import random
import satelliteParam as P

class satelliteDynamics:
    def __init__(self):
        # Initial state conditions
        self.state = np.array([
            [P.theta0],  # initial base angle
            [P.phi0],  # initial panel angle
            [P.thetadot0],  # initial angular velocity of base
            [P.phidot0],  # initial angular velocity of panel
        ])
        self.Ts = P.Ts
        #################################################
        # The parameters for any physical system are never known exactly.  Feedback
        # systems need to be designed to be robust to this uncertainty.  In the simulation
        # we model uncertainty by changing the physical parameters by a uniform random variable
        # that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
        # may change by up to 20%.  A different parameter value is chosen every time the simulation
        # is run.
        alpha = 0.2  # Uncertainty parameter
        self.Js = P.Js * (1.+alpha*(2.*np.random.rand()-1.))  # inertia of base
        self.Jp = P.Jp * (1.+alpha*(2.*np.random.rand()-1.))  # inertia of panel
        self.k = P.k * (1.+alpha*(2.*np.random.rand()-1.))    # spring coefficient
        self.b = P.b * (1.+alpha*(2.*np.random.rand()-1.))    # Damping coefficient, Ns
        self.torque_limit = P.tau_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input torque
        u = self.saturate(u, self.torque_limit)
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, u):
        # Return xdot = f(x,u)
        theta = state.item(0)
        phi = state.item(1)
        thetadot = state.item(2)
        phidot = state.item(3)
        tau = u
        # The equations of motion.
        M = np.array([[self.Js, 0],
                       [0, self.Jp]])
        C = np.array([[tau - self.b*(thetadot-phidot)-self.k*(theta-phi)],
                       [-self.b*(phidot-thetadot)-self.k*(phi-theta)]])
        tmp = np.linalg.inv(M) @ C
        thetaddot = tmp.item(0)
        phiddot = tmp.item(1)
        # build xdot and return
        xdot = np.array([[thetadot], [phidot], [thetaddot], [phiddot]])
        return xdot

    def h(self):
        # return y = h(x)
        theta = self.state.item(0)
        phi = self.state.item(1)
        y = np.array([[theta], [phi]])
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
            u = limit*np.sign(u)
        return u