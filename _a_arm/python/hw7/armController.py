import numpy as np
import armParamHW7 as P
import sys
sys.path.append('..')  # add parent directory
import armParam as P0

class armController:
    def __init__(self):
        # Instantiates the PD object
        self.kp = P.kp
        self.kd = P.kd
        self.limit = P0.tau_max

    def update(self, theta_r, x):
        theta = x.item(0)
        thetadot = x.item(1)
        # feedback linearized torque
        tau_fl = P0.m * P0.g * (P0.ell / 2.0) * np.cos(theta)
        # equilibrium torque around theta_e = 0
        theta_e = 0.0
        tau_e = P0.m * P0.g * P0.ell/2.0 * np.cos(theta_e)
        # compute the linearized torque using PD control
        tau_tilde = self.kp * (theta_r - theta) - self.kd * thetadot
        # compute total torque
        tau = tau_fl + tau_tilde
        #tau = tau_e + tau_tilde
        # always saturate to protect hardware
        tau = self.saturate(tau)
        return tau

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







