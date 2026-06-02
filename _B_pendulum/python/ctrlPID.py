import numpy as np
import pendulumParam as P

class ctrlPID:
    def __init__(self):
        # dirty derivative parameters
        self.sigma = 0.05  # cutoff freq for dirty derivative
        
        ####################################################
        #       PD Control: Time Design Strategy
        ####################################################
        # tuning parameters
        tr_th = 0.2  # Rise time: inner loop (theta)
        zeta_th = 0.707  # Damping Coefficient: inner loop (theta)
        M = 10.0  # Time scale separation between inner & outer loop
        zeta_z = 0.707  # Damping Coefficient: outer loop (z)
        self.ki_z = -0.05  # select integrator gain

        # saturation limits
        self.theta_max = 30.0 * np.pi / 180.0  # Max theta, rads
        #---------------------------------------------------
        #                    Inner Loop
        #---------------------------------------------------
        # parameters of the open loop transfer function
        b0_th = -1.0 / (P.m1 * (P.ell / 6.0) \
            + P.m2 * (2.0 * P.ell / 3.0))
        a1_th = 0.0
        a0_th = -(P.m1 + P.m2) * P.g / (P.m1 * (P.ell / 6.0) \
            + P.m2 * (2.0 * P.ell / 3.0))
        
        # coefficients for desired inner loop
        wn_th = 2.2 / tr_th     # Natural frequency
        alpha1_th = 2.0 * zeta_th * wn_th
        alpha0_th = wn_th**2

        # compute gains
        self.kp_th = (alpha0_th - a0_th) / b0_th
        self.kd_th = (alpha1_th - a1_th) / b0_th
        DC_gain = self.kp_th / ((P.m1 + P.m2) * P.g + self.kp_th)

        #---------------------------------------------------
        #                    Outer Loop
        #---------------------------------------------------
        # coefficients for desired outer loop
        tr_z = M * tr_th  # desired rise time, s
        wn_z = 2.2 / tr_z  # desired natural frequency

        # compute gains
        a  = -(wn_z**2) * np.sqrt(2.0 * P.ell / (3.0 * P.g))
        b = (a - 2.0 * zeta_z * wn_z) \
            * np.sqrt(2.0 * P.ell / (3.0 * P.g))
        self.kd_z = b / (1 - b)
        self.kp_z = a * (1 + self.kd_z)

        # print control gains to terminal        
        print('DC_gain', DC_gain)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)
        print('kp_z: ', self.kp_z)
        print('ki_z: ', self.ki_z)
        print('kd_z: ', self.kd_z)

        #---------------------------------------------------
        # initialize zero canceling filter
        #---------------------------------------------------
        self.filter = zeroCancelingFilter(DC_gain)

        #---------------------------------------------------
        # initialize variables for integrator and differentiators
        #---------------------------------------------------
        self.integrator_z = 0.
        self.error_z_prev = 0.
        self.z_dot = P.zdot0
        self.z_prev = P.z0
        self.theta_dot = P.thetadot0
        self.theta_prev = P.theta0
        
    def update(self, z_r, y):
        z = y[0, 0]
        theta = y[1, 0]
        #---------------------------------------------------
        # Update Outer Loop (z-control)
        #---------------------------------------------------
        # Compute the error in z
        error_z = z_r - z

        # differentiate z
        self.z_dot = (2.0*self.sigma - P.Ts) / (2.0*self.sigma + P.Ts) * self.z_dot \
            + (2.0 / (2.0*self.sigma + P.Ts)) * ((z - self.z_prev))
        
        # if z_dot is small, integrate z
        if np.abs(self.z_dot) < 0.07:
            self.integrator_z = self.integrator_z \
                + (P.Ts / 2) * (error_z + self.error_z_prev)
        
        # PID control - unsaturated
        theta_r_unsat = self.kp_z * error_z \
                + self.ki_z * self.integrator_z \
                - self.kd_z * self.z_dot
        
        # saturate theta_r
        theta_r = saturate(theta_r_unsat, self.theta_max)

        # integrator anti - windup
        # if self.ki_z != 0.0:
        #     self.integrator_z = self.integrator_z \
        #         + P.Ts / self.ki_z * (theta_r - theta_r_unsat)


        #---------------------------------------------------
        # zero canceling filter applied to theta_r to cancel
        # left-half plane zero and DC-gain
        #---------------------------------------------------
        theta_r = self.filter.update(theta_r)

        #---------------------------------------------------
        # Update Inner Loop (theta-control)
        #---------------------------------------------------
        # Compute the error in theta
        error_th = theta_r - theta

        # differentiate theta
        self.theta_dot = (2.0*self.sigma - P.Ts) / (2.0*self.sigma + P.Ts) * self.theta_dot \
            + (2.0 / (2.0*self.sigma + P.Ts)) * ((theta - self.theta_prev))
        
         # PD control on theta
        F_unsat = self.kp_th * error_th \
            - self.kd_th * self.theta_dot
        
        # saturate the force
        F = saturate(F_unsat, P.F_max)

        # update delayed variables
        self.error_z_prev = error_z
        self.z_prev = z
        self.theta_prev = theta

        # return computed force
        return F


class zeroCancelingFilter:
    def __init__(self, DC_gain):
        self.a = -3.0 / (2.0 * P.ell * DC_gain)
        self.b = np.sqrt(3.0 * P.g / (2.0 * P.ell))
        self.state = 0.0

    def update(self, input):
        # integrate using RK1
        self.state = self.state \
            + P.Ts * (-self.b * self.state + self.a * input)
        return self.state


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u





