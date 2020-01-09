import numpy as np
import armParamHW11 as P

class armController:
    # dirty derivatives to estimate thetadot
    def __init__(self):
        self.K = P.K  # state feedback gain
        self.kr = P.kr  # Input gain
        self.limit = P.tau_max  # Maxiumum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, theta_r, x):
        theta = x.item(0)
        thetadot = x.item(1)

        # compute feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)

        # Compute the state feedback controller
        tau_tilde = -self.K @ x + self.kr * theta_r

        # compute total torque
        tau = self.saturate(tau_fl + tau_tilde)
        return tau

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

