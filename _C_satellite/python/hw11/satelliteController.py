import numpy as np
import satelliteParamHW11 as P

class satelliteController:
    # dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.phi_dot = 0.0  # derivative of phi
        self.theta_dot = 0.0  # derivative of theta
        self.phi_d1 = 0.  # angle phi delayed by 1 sample
        self.theta_d1 = 0.0  # Angle theta delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.kr = P.kr  # Input gain
        self.limit = P.tau_max  # Maxiumum torque
        self.beta = P.beta  # dirty derivative gain
        self.Ts = P.Ts  # sample rate of controller

    def update(self, phi_r, y):
        theta = y.item(0)
        phi = y.item(1)

        # differentiate z and theta
        self.differentiatePhi(phi)
        self.differentiateTheta(theta)
        # Construct the state
        x = np.array([[theta], [phi],
                      [self.theta_dot], [self.phi_dot]])
        # Compute the state feedback controller
        tau_unsat = -self.K @ x + self.kr * phi_r
        tau = self.saturate(tau_unsat)
        return tau

    def differentiatePhi(self, phi):
        self.phi_dot = self.beta*self.phi_dot \
            + (1-self.beta)/self.Ts * (phi - self.phi_d1)
        self.phi_d1 = phi

    def differentiateTheta(self, theta):
        self.theta_dot = self.beta*self.theta_dot \
            + (1-self.beta)/self.Ts * (theta - self.theta_d1)
        self.theta_d1 = theta

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

