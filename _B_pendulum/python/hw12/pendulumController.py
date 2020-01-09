import numpy as np
import pendulumParamHW12 as P

class pendulumController:
    def __init__(self):
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.ki = P.ki  # integrator gain
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, z_r, x):
        z = x.item(0)
        # integrate error
        error = z_r - z
        self.integrateError(error)
        # Compute the state feedback controller
        F_unsat = -self.K @ x - self.ki*self.integrator
        F_sat = self.saturate(F_unsat)
        return F_sat.item(0)

    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

