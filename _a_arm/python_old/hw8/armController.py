import numpy as np
import armParamHW8 as P
import sys
sys.path.append('..')  # add parent directory
import armParam as P0
from PDControl import PDControl


class armController:

    def __init__(self):
        # Instantiates the PD object
        self.thetaCtrl = PDControl(P.kp, P.kd, P0.tau_max, P.beta, P.Ts)
        self.limit = P0.tau_max

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        theta_r = y_r[0]
        theta = y[0]

        # compute equilibrium torque tau_e
        tau_e = P0.m * P0.g * (P0.ell / 2.0) * np.cos(theta)
        # compute the linearized torque using PD
        tau_tilde = self.thetaCtrl.PD(theta_r, theta, False)
        # compute total torque
        tau = tau_e + tau_tilde
        tau = self.saturate(tau)
        return [tau]

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







