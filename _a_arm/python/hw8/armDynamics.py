import numpy as np 
import random
import armParam as P


class armDynamics:
    '''
        Model the physical system
    '''

    def __init__(self):
        # Initial state conditions
        self.state = np.array([
            [P.theta0],      # initial angle
            [P.thetadot0]
        ])  # initial angular rate
        self.output = 0.0
        #################################################
        # The parameters for any physical system are never known exactly.  Feedback
        # systems need to be designed to be robust to this uncertainty.  In the simulation
        # we model uncertainty by changing the physical parameters by a uniform random variable
        # that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
        # may change by up to 20%.  A different parameter value is chosen every time the simulation
        # is run.
        alpha = 0.0  # Uncertainty parameter
        self._m = P.m * (1.+alpha*(2.*np.random.rand()-1.))  # Mass of the arm, kg
        self._ell = P.ell * (1.+alpha*(2.*np.random.rand()-1.))  # Length of the arm, m
        self._b = P.b * (1.+alpha*(2.*np.random.rand()-1.))  # Damping coefficient, Ns
        self._g = P.g  # the gravity constant is well known and so we don't change it.
        self._Ts = P.Ts  # sample rate at which the dynamics are propagated
        self.torque_limit = P.tau_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input torque
        u = self.saturate(u, self.torque_limit)
        self._rk4_step(u)  # propagate the state by one time sample
        y = self._h()  # return the corresponding output
        self.output = y
        return y

    def _rk1_step(self, u):
        # Integrate ODE using Runge-Kutta RK1 algorithm
        self.state += self._Ts * self._f(self.state, u)

    def _rk2_step(self, u):
        # Integrate ODE using Runge-Kutta RK2 algorithm
        F1 = self._f(self.state, u)
        F2 = self._f(self.state + self._Ts / 2 * F1, u)
        self.state += self._Ts / 2 * (F1 + F2)

    def _rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self._f(self.state, u)
        F2 = self._f(self.state + self._Ts / 2 * F1, u)
        F3 = self._f(self.state + self._Ts / 2 * F2, u)
        F4 = self._f(self.state + self._Ts * F3, u)
        self.state += self._Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def _f(self, state, tau):
        # Return xdot = f(x,u), the system state update equations
        # re-label states for readability
        theta = state.item(0)
        thetadot = state.item(1)
        xdot = np.array([
            [thetadot],
            [(3.0/self._m/self._ell**2)*(tau - self._b*thetadot - self._m*self._g*self._ell/2.0*np.cos(theta))],
        ])
        return xdot

    def _h(self):
        # return the output equations
        # could also use input u if needed
        theta = self.state.item(0)
        thetadot = self.state.item(1)
        y = np.array([
            [theta],
        ])
        return y

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
        return u