import numpy as np
import pendulumParamHW11 as P

class pendulumController:
    def __init__(self):
        self.K = P.K  # state feedback gain
        self.kr = P.kr  # Input gain
        self.limit = P.F_max  # Maxiumum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, z_r, x):
        # Compute the state feedback controller
        F_unsat = -self.K @ x + self.kr * z_r
        F_sat = self.saturate(F_unsat)
        return F_sat.item(0)

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

