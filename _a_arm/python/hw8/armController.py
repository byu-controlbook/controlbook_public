import numpy as np
import armParamHW8 as P
import sys
sys.path.append('..')  # add parent directory
import armParam as P0


class armController:

    def __init__(self):
        # Instantiates the PD object
        self.kp = P.kp
        self.kd = P.kd
        self.tau_max = P.tau_max

    def update(self, theta_r, state):
        theta = state.item(0)
        thetadot = state.item(1)

        # compute feedback linearizing torque tau_fl
        tau_fl = P0.m * P0.g * (P0.ell / 2.0) * np.cos(theta)
        # compute the linearized torque using PD
        tau_tilde = self.kp * (theta_r - theta) - self.kd * thetadot
        # compute total torque
        tau = tau_fl + tau_tilde
        tau = self.saturate(tau, self.tau_max)
        return tau

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
        return u







