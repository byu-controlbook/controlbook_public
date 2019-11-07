import numpy as np
import satelliteParamHW11 as P

class satelliteController:
    def __init__(self):
        self.K = P.K  # state feedback gain
        self.kr = P.kr  # Input gain
        self.limit = P.tau_max  # Maxiumum torque
        self.Ts = P.Ts  # sample rate of controller

    def update(self, phi_r, x):
        # Compute the state feedback controller
        tau_unsat = -self.K @ x + self.kr * phi_r
        tau = self.saturate(tau_unsat.item(0))
        return tau

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

