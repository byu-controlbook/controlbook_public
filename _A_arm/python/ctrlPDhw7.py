import numpy as np
import armParam as P

class ctrlPD:
    def __init__(self):
        # PD gains
        self.kp = 0.18
        self.kd = 0.095
        print('kp: ', self.kp)
        print('kd: ', self.kd)

    def update(self, theta_r, x):
        theta = x[0][0]
        thetadot = x[1][0]
        # feedback linearized torque
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)
        # equilibrium torque around theta_e = 0
        theta_e = 0.0
        tau_e = P.m * P.g * P.ell / 2.0 * np.cos(theta_e)
        # compute the linearized torque using PD control
        tau_tilde = self.kp * (theta_r - theta) - self.kd * thetadot
        # compute total torque
        tau = tau_fl + tau_tilde
        #tau = tau_e + tau_tilde
        # always saturate to protect hardware
        tau = saturate(tau, P.tau_max)
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u







