import numpy as np
import armParam as P


class ctrlPID:
    def __init__(self):
        #  tuning parameters
        tr = 0.6  
        zeta = 0.90
        self.ki = 0.2  # integrator gain

        # desired natural frequency
        #wn = 2.2 / tr
        wn = 0.5*np.pi/(tr*np.sqrt(1-zeta**2))

        # compute PD gains
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2
        self.kp = alpha0 * (P.m * P.ell**2) / 3.0
        self.kd = (P.m * P.ell**2) \
            / 3.0 * (alpha1 - 3.0 * P.b / (P.m * P.ell**2))

        # print gains
        print('kp: ', self.kp)
        print('ki: ', self.ki)
        print('kd: ', self.kd)       
        
        # dirty derivative gain
        self.sigma = 0.05  

        #----------------------------------------------------------
        # variables for integrator and differentiator
        self.theta_dot = P.thetadot0  # estimated derivative of theta
        self.theta_dot_prev = P.thetadot0
        self.theta_prev = P.theta0  # theta delayed by one sample
        self.error_dot = 0.0  # estimated derivative of error
        self.error_prev = 0.0  # Error delayed by one sample
        self.integrator = 0.0  # integrator

    def update(self, theta_r, y):
        theta = y[0, 0]

        # compute feedback linearized torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)

        # compute the linearized torque using PID
        # Compute the current error
        error = theta_r - theta

        # differentiate theta
        self.theta_dot = (2.0*self.sigma - P.Ts) / (2.0*self.sigma + P.Ts) * self.theta_dot_prev \
            + (2.0 / (2.0*self.sigma + P.Ts)) * ((theta - self.theta_prev))
        

        
        # Anti-windup scheme: only integrate theta when theta_dot is small
        if abs(self.theta_dot < 0.08):
            self.integrator = self.integrator \
                + (P.Ts / 2) * (error + self.error_prev)
            
        # PID control
        tau_tilde = self.kp * error \
            + self.ki * self.integrator \
                - self.kd * self.theta_dot
        
        # compute total torque
        tau_unsat = tau_fl + tau_tilde
        tau = saturate(tau_unsat, P.tau_max)

        # update delayed variables
        self.error_prev = error
        self.theta_prev = theta
        self.theta_dot_prev = self.theta_dot 
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u








