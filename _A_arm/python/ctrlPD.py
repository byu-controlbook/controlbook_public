import numpy as np
import armParam as P


class ctrlPD:
    def __init__(self):
        #  tuning parameters
        #tr = 0.8 # part (a)
        tr = 0.4  # tuned for faster rise time before saturation.
        zeta = 0.707
        # desired natural frequency
        wn = 2.2 / tr
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2
        # compute PD gains
        self.kp = alpha0*(P.m * P.ell**2) / 3.0
        self.kd = (P.m * P.ell**2) \
                    / 3.0 * (alpha1 - 3.0 * P.b / (P.m * P.ell**2))
        print('kp: ', self.kp)
        print('kd: ', self.kd)        

    def update(self, theta_r, state):
        theta = state[0][0]
        thetadot = state[1][0]
        # compute feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)
        # compute the linearized torque using PD
        tau_tilde = self.kp * (theta_r - theta) \
                    - self.kd * thetadot
        # compute total torque
        tau = tau_fl + tau_tilde
        tau = saturate(tau, P.tau_max)
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u








