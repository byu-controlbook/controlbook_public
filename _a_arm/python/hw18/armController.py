import sys
sys.path.append('..')  # add parent directory
import armParam as P
import loopshape_arm as L
import numpy as np
from transferFunction import transferFunction

class armController:
    def __init__(self):
        self.prefilter = transferFunction(L.F_num, L.F_den, P.Ts)
        self.control = transferFunction(L.C_num, L.C_den, P.Ts)
        self.limit = P.tau_max  # Maximum force

    def update(self, theta_r, y):
        theta = y[0]

         # prefilter the reference
        theta_r_filtered = self.prefilter.update(theta_r)

        # define error and update controller
        error = theta_r_filtered - theta
        tau_tilde = self.control.update(error)

        # compute feedback linearized torque
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)

        # compute total torque
        tau = self.saturate(tau_fl + tau_tilde)
        return tau

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

