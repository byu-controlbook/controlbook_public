import numpy as np
import satelliteParamHW12 as P

class satelliteController:
    def __init__(self):
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # integrator gain
        self.limit = P.tau_max       # Maxiumum torque
        self.Ts = P.Ts               # sample rate of controller

    def update(self, phi_r, x):
        theta = x.item(0)
        phi = x.item(1)

        # integrate error
        error = phi_r - phi
        self.integrateError(error)

        # Compute the state feedback controller
        tau_unsat = -self.K @ x - self.ki*self.integrator
        tau = self.saturate(tau_unsat.item(0))

        return tau

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts / 2.0) \
                          * (error + self.error_d1)

        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

