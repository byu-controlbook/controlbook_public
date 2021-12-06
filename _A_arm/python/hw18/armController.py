import sys
sys.path.append('..')  # add parent directory
import armParam as P
import loopshape_arm as L
import numpy as np
from discreteFilter import discreteFilter
from transferFunction import transferFunction

class armController:
    def __init__(self, method="state_space"):
        if method == "state_space":
            self.prefilter = transferFunction(L.F_num, L.F_den, P.Ts)
            self.control = transferFunction(L.C_num, L.C_den, P.Ts)

        elif method == "digital_filter":
            self.prefilter = discreteFilter(L.F.num, L.F.den, P.Ts)
            self.control = discreteFilter(L.C.num, L.C.den, P.Ts)

        self.limit = P.tau_max  # Maximum torque
        self.Ts = P.Ts  # sample rate of
        self.method = method


    def update(self, theta_r, y):
        theta = y.item(0)
        # prefilter
        theta_r_filtered = self.prefilter.update(theta_r)
        # filtered error signal
        error = theta_r_filtered - theta
        # update controller
        tau_tilde = self.control.update(error)
        # compute equilibrium torque tau_e
        tau_e = P.m * P.g * (P.ell / 2.0) * np.cos(theta)
        # compute total torque
        tau = self.saturate(tau_e + tau_tilde)
        return tau


    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u