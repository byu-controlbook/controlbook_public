import numpy as np 
import random
import armParam as P


class armDynamics:
    '''
        Model the physical system
    '''

    def __init__(self):
        # Initial state conditions
        self.state = np.matrix([[P.theta0],      # initial angle
                                [P.thetadot0]])  # initial angular rate
        #################################################
        # The parameters for any physical system are never known exactly.  Feedback
        # systems need to be designed to be robust to this uncertainty.  In the simulation
        # we model uncertainty by changing the physical parameters by a uniform random variable
        # that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
        # may change by up to 20%.  A different parameter value is chosen every time the simulation
        # is run.
        alpha = 0.2  # Uncertainty parameter
        self.m = P.m * (1.+alpha*(2.*np.random.rand()-1.))  # Mass of the arm, kg
        self.ell = P.ell * (1.+alpha*(2.*np.random.rand()-1.))  # Length of the arm, m
        self.b = P.b * (1.+alpha*(2.*np.random.rand()-1.))  # Damping coefficient, Ns
        self.g = P.g  # the gravity constant is well known and so we don't change it.
        self.Ts = P.Ts  # sample rate at which the dynamics are propagated

    def propagateDynamics(self, u):
        '''
            Integrate the differential equations defining dynamics
            P.Ts is the time step between function calls.
            u contains the system input(s).
        '''
        # Integrate ODE using Runge-Kutta RK4 algorithm
        k1 = self.derivatives(self.state, u)
        k2 = self.derivatives(self.state + self.Ts/2*k1, u)
        k3 = self.derivatives(self.state + self.Ts/2*k2, u)
        k4 = self.derivatives(self.state + self.Ts*k3, u)
        self.state += self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)

    def derivatives(self, state, u):
        '''
            Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
        '''
        # re-label states and inputs for readability
        theta = state.item(0)
        thetadot = state.item(1)
        tau = u[0]
        # The equations of motion.
        thetaddot = (3.0/self.m/self.ell**2)*(tau - self.b*thetadot - self.m*self.g*self.ell/2.0*np.cos(theta))

        # build xdot and return
        xdot = np.matrix([[thetadot], [thetaddot]])
        return xdot

    def outputs(self):
        '''
            Returns the measured outputs as a list
            [theta] with added Gaussian noise
        '''
        # re-label states for readability
        theta = self.state.item(0)
        # add Gaussian noise to outputs
        theta_m = theta + random.gauss(0, 0.001)
        # return measured outputs
        return [theta_m]

    def states(self):
        '''
            Returns all current states as a list
        '''
        return self.state.T.tolist()[0]