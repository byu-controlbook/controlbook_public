import numpy as np 
import random
import pendulumParam as P


class pendulumDynamics:
    '''
        Model the physical system
    '''

    def __init__(self):
        # Initial state conditions
        self.state = np.array([
            [P.z0],          # z initial position
            [P.theta0],      # Theta initial orientation
            [P.zdot0],       # zdot initial velocity
            [P.thetadot0],
        ])  # Thetadot initial velocity
        self._Ts = P.Ts
        #################################################
        # The parameters for any physical system are never known exactly.  Feedback
        # systems need to be designed to be robust to this uncertainty.  In the simulation
        # we model uncertainty by changing the physical parameters by a uniform random variable
        # that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
        # may change by up to 20%.  A different parameter value is chosen every time the simulation
        # is run.
        alpha = 0.2  # Uncertainty parameter
        self._m1 = P.m1 * (1.+alpha*(2.*np.random.rand()-1.))  # Mass of the pendulum, kg
        self._m2 = P.m2 * (1.+alpha*(2.*np.random.rand()-1.))  # Mass of the cart, kg
        self._ell = P.ell * (1.+alpha*(2.*np.random.rand()-1.))  # Length of the rod, m
        self._b = P.b * (1.+alpha*(2.*np.random.rand()-1.))  # Damping coefficient, Ns
        self._g = P.g  # the gravity constant is well known and so we don't change it.
        self.force_limit = P.F_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input force
        u = self.saturate(u, self.force_limit)
        self._rk4_step(u)  # propagate the state by one time sample
        y = self._h()  # return the corresponding output
        return y

    def _rk1_step(self, u):
        # Integrate ODE using Runge-Kutta RK1 algorithm
        self.state += self._Ts * self._f(self.state, u)

    def _rk2_step(self, u):
        # Integrate ODE using Runge-Kutta RK2 algorithm
        F1 = self._f(self.state, u)
        F2 = self._f(self.state + self._Ts * F1, u)
        self.state += self._Ts / 2 * (F1 + F2)

    def _rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self._f(self.state, u)
        F2 = self._f(self.state + self._Ts / 2 * F1, u)
        F3 = self._f(self.state + self._Ts / 2 * F2, u)
        F4 = self._f(self.state + self._Ts * F3, u)
        self.state += self._Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)


    def _f(self, state, u):
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
        M = np.matrix([[self._m1+self._m2, self._m1*(self._ell/2.0)*np.cos(theta)],
                       [self._m1*(self._ell/2.0)*np.cos(theta), self._m1*(self._ell**2/3.0)]])
        C = np.matrix([[self._m1*(self._ell/2.0)*thetadot**2*np.sin(theta) + F - self._b*zdot],
                       [self._m1*self._g*(self._ell/2.0)*np.sin(theta)]])
        tmp = np.linalg.inv(M)*C
        zddot = tmp.item(0)
        thetaddot = tmp.item(1)
        # build xdot and return
        xdot = np.array([[zdot], [thetadot], [zddot], [thetaddot]])
        return xdot

    def _h(self):
        '''
            Returns the measured outputs as a list
            [z, theta] with added Gaussian noise
        '''
        # re-label states for readability
        z = self.state.item(0)
        theta = self.state.item(1)
        # # add Gaussian noise to outputs
        # z_m = z + random.gauss(0, 0.01)
        # theta_m = theta + random.gauss(0, 0.001)
        # return measured outputs
        y = np.array([
            [z],
            [theta],
        ])
        return y

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
        return u