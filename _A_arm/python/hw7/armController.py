import numpy as np
import hw7.armParamHW7 as P
import armParam as P0

class armController:
    def __init__(self):

        # Instantiates the PD object
        self.kp = P.kp
        self.kd = P.kd
        self.limit = P0.tau_max

    def update(self, theta_r, x):
        theta = x[0,0]
        thetadot = x[1,0]

        # feedback linearized torque
        tau_fl = P0.m * P0.g * (P0.ell / 2.0) * np.cos(theta)
        #tau_eq = P0.m * P0.g * (P0.ell / 2.0) * np.cos(P0.theta0)

        # compute the linearized torque using PD control
        tau_tilde = self.kp * (theta_r - theta) - self.kd * thetadot

        # compute total torque
        tau = tau_fl + tau_tilde
        #tau = tau_eq + tau_tilde

        # always saturate to protect hardware
        tau = self.saturate(tau)

        return tau

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







