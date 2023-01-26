import numpy as np
import armParam as P


class ctrlPID:
    def __init__(self):
        #  tuning parameters
        #tr = 0.8 # part (a)
        tr = 0.6  # tuned for fastest possible rise time before saturation.
        zeta = 0.90
        self.ki = 0.1  # integrator gain
        # desired natural frequency
        wn = 2.2 / tr
        #wn = 0.5*np.pi/(tr*np.sqrt(1-zeta**2))
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2
        # compute PD gains
        self.kp = alpha0 * (P.m * P.ell**2) / 3.0
        self.kd = (P.m * P.ell**2) / 3.0 * (alpha1 - 3.0 * P.b / (P.m * P.ell**2))
        print('kp: ', self.kp)
        print('ki: ', self.ki)
        print('kd: ', self.kd)       
        # dirty derivative gains
        self.sigma = 0.05  
        self.beta = (2.0 * self.sigma - P.Ts) / (2.0 * self.sigma + P.Ts)  # dirty derivative gain
        #----------------------------------------------------------
        # variables for integrator and differentiator
        self.theta_dot = 0.0             # estimated derivative of y
        self.theta_d1 = 0.0              # Signal y delayed by one sample
        self.error_dot = 0.0         # estimated derivative of error
        self.error_d1 = 0.0          # Error delayed by one sample
        self.integrator = 0.0        # integrator

    def update(self, theta_r, y):
        theta = y[0][0]
        # compute feedback linearized torque tau_fl
        #tau_e = P0.m * P0.g * (P0.ell / 2.0) * np.cos(0.0)
        # compute feedback linearized torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)
        # compute the linearized torque using PID
        # Compute the current error
        error = theta_r - theta
        # integrate error
        self.integrator = self.integrator + (P.Ts / 2) * (error + self.error_d1)
        # differentiate theta
        self.theta_dot = self.beta * self.theta_dot \
                             + (1 - self.beta) * ((theta - self.theta_d1) / P.Ts)
        # PID control
        tau_tilde = self.kp * error + self.ki * self.integrator - self.kd * self.theta_dot
        # compute total torque
        tau_unsat = tau_fl + tau_tilde
        tau = saturate(tau_unsat, P.tau_max)
        # integrator anti - windup
        if self.ki != 0.0:
            self.integrator = self.integrator + P.Ts / self.ki * (tau - tau_unsat)
        # update delayed variables
        self.error_d1 = error
        self.theta_d1 = theta
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u








