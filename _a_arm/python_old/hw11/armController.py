import numpy as np
import armParamHW11 as P

class armController:
    # state feedback control using dirty derivatives to estimate thetadot
    def __init__(self):
        self.theta_dot = 0.0          # derivative of theta
        self.theta_d1 = 0.0          # Angle theta delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.kr = P.kr               # Input gain
        self.limit = P.tau_max         # Maxiumum force
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        theta_r = y_r[0]
        theta = y[0]

        # differentiate theta
        self.differentiateTheta(theta)

        # Construct the state
        x = np.matrix([[theta], [self.theta_dot]])

        # compute equilibrium torque tau_e
        tau_e = P.m * P.g * (P.ell / 2.0) * np.cos(theta)

        # Compute the state feedback controller
        tau_tilde = -self.K*x + self.kr*theta_r

        # compute total torque
        tau = self.saturate(tau_e + tau_tilde)
        return [tau.item(0)]

    def differentiateTheta(self, theta):
        '''
            differentiate theta
        '''
        self.theta_dot = self.beta*self.theta_dot + (1-self.beta)*((theta - self.theta_d1) / self.Ts)
        self.theta_d1 = theta

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

