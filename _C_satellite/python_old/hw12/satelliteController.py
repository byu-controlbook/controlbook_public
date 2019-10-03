import numpy as np
import satelliteParamHW12 as P

class satelliteController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.phi_dot = 0.0           # derivative of phi
        self.theta_dot = 0.0         # derivative of theta
        self.phi_d1 = 0.             # angle phi delayed by 1 sample
        self.theta_d1 = 0.0          # Angle theta delayed by 1 sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # integrator gain
        self.limit = P.tau_max       # Maxiumum torque
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        phi_r = y_r[0]
        theta = y[0]
        phi = y[1]

        # differentiate z and theta
        self.differentiatePhi(phi)
        self.differentiateTheta(theta)

        # integrate error
        error = phi_r - phi
        self.integrateError(error)

        # Construct the state
        x = np.matrix([[theta], [phi], [self.theta_dot], [self.phi_dot]])

        # Compute the state feedback controller
        tau_unsat = -self.K*x - self.ki*self.integrator

        tau = self.saturate(tau_unsat)
        return [tau.item(0)]

    def differentiatePhi(self, phi):
        '''
            differentiate z
        '''
        self.phi_dot = self.beta*self.phi_dot + (1-self.beta)*((phi - self.phi_d1) / self.Ts)
        self.phi_d1 = phi

    def differentiateTheta(self, theta):
        '''
            differentiate theta
        '''
        self.theta_dot = self.beta*self.theta_dot + (1-self.beta)*((theta - self.theta_d1) / self.Ts)
        self.theta_d1 = theta

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

