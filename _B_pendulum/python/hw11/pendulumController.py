import numpy as np
import pendulumParamHW11 as P

class pendulumController:
    # dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.z_dot = 0.0              # derivative of z
        self.theta_dot = 0.0          # derivative of theta
        self.z_d1 = 0.0              # Position z delayed by 1 sample
        self.theta_d1 = 0.0          # Angle theta delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.kr = P.kr               # Input gain
        self.limit = P.F_max         # Maxiumum force
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def update(self, z_r, y):
        z = y.item(0)
        theta = y.item(1)
        # differentiate z and theta
        self.differentiateZ(z)
        self.differentiateTheta(theta)
        # Construct the state
        x = np.array([[z], [theta], [self.z_dot], [self.theta_dot]])
        # Compute the state feedback controller
        F_unsat = -self.K @ x + self.kr * z_r
        F_sat = self.saturate(F_unsat)
        return F_sat

    def differentiateZ(self, z):
        self.z_dot = self.beta*self.z_dot \
            + (1-self.beta)/self.Ts * (z - self.z_d1)
        self.z_d1 = z

    def differentiateTheta(self, theta):
        self.theta_dot = self.beta*self.theta_dot \
            + (1-self.beta)/self.Ts * (theta - self.theta_d1)
        self.theta_d1 = theta

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

