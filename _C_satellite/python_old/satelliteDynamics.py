import numpy as np 
import random
import satelliteParam as P


class satelliteDynamics:
    '''
        Model the physical system
    '''

    def __init__(self):
        # Initial state conditions
        self.state = np.matrix([[P.theta0],     # initial base angle
                                [P.phi0],       # initial panel angle
                                [P.thetadot0],  # initial angular velocity of base
                                [P.phidot0]])   # nitial angular velocity of panel
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

    def propagateDynamics(self, u):
        '''
            Integrate the differential equations defining dynamics
            P.Ts is the time step between function calls.
            u contains the system input(s).
        '''
        # Integrate ODE using Runge-Kutta RK4 algorithm
        k1 = self.derivatives(self.state, u)
        k2 = self.derivatives(self.state + P.Ts/2*k1, u)
        k3 = self.derivatives(self.state + P.Ts/2*k2, u)
        k4 = self.derivatives(self.state + P.Ts*k3, u)
        self.state += P.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)

    def derivatives(self, state, u):
        '''
            Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
        '''
        # re-label states and inputs for readability
        theta = state.item(0)
        phi = state.item(1)
        thetadot = state.item(2)
        phidot = state.item(3)
        tau = u[0]
        # The equations of motion.
        M = np.matrix([[self.Js, 0],
                       [0, self.Jp]])
        C = np.matrix([[tau - self.b*(thetadot-phidot)-self.k*(theta-phi)],
                       [-self.b*(phidot-thetadot)-self.k*(phi-theta)]])
        tmp = np.linalg.inv(M)*C
        thetaddot = tmp.item(0)
        phiddot = tmp.item(1)
        # build xdot and return
        xdot = np.matrix([[thetadot], [phidot], [thetaddot], [phiddot]])
        return xdot

    def outputs(self):
        '''
            Returns the measured outputs as a list
            [z, theta] with added Gaussian noise
        '''
        # re-label states for readability
        theta = self.state.item(0)
        phi = self.state.item(1)
        # add Gaussian noise to outputs
        theta_m = theta + random.gauss(0, 0.001)
        phi_m = phi + random.gauss(0, 0.001)
        # return measured outputs
        return [theta_m, phi_m]

    def states(self):
        '''
            Returns all current states as a list
        '''
        return self.state.T.tolist()[0]