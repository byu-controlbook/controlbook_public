import numpy as np
import armParamHW12 as P

class armController:
    def __init__(self):
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.ki = P.ki  # Input gain
        self.limit = P.tau_max  # Maxiumum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, theta_r, x):
        theta = x.item(0)

        # integrate error
        error = theta_r - theta
        self.integrateError(error)

        # compute feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)

        # Compute the state feedback controller
        tau_tilde = -self.K @ x - self.ki*self.integrator

        # compute total torque
        tau = self.saturate(tau_fl + tau_tilde)
        return tau

    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

