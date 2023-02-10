import numpy as np
import satelliteParam as P


class ctrlPID:
    def __init__(self):
        self.sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * self.sigma - P.Ts) \
            / (2 * self.sigma + P.Ts)  
        ####################################################
        #       PD Control: Time Design Strategy
        ####################################################
        # tuning parameters
        tr_th = 0.4  # rise time for inner loop
        zeta_th = 0.9  # damping ratio for inner loop
        M = 15.0  # Time scale separation between inner & outer loop
        zeta_phi = 0.9  # damping ratio for outer loop
        self.ki_phi = 0.25  # integral gain for outer loop
        # saturation limits
        self.theta_max = 30.0*np.pi/180.0  
            # maximum commanded base angle
        #---------------------------------------------------
        #                    Inner Loop
        #---------------------------------------------------
        # PD design for inner loop
        wn_th = 2.2 / tr_th
        self.kp_th = wn_th**2 * (P.Js + P.Jp)
        self.kd_th = 2 * zeta_th * wn_th * (P.Js + P.Jp)
        # DC gain for inner loop
        k_DC_th = 1
        #---------------------------------------------------
        #                    Outer Loop
        #---------------------------------------------------
        # PD design for outer loop
        tr_phi = M * tr_th  # rise time for outer loop
        wn_phi =2.2 / tr_phi
        AA = np.array([
                [P.k * k_DC_th, \
                    -P.b * k_DC_th * wn_phi**2],
                [P.b * k_DC_th, \
                    P.k * k_DC_th \
                        - 2 * zeta_phi * wn_phi * P.b * k_DC_th]])    
        bb = np.array([
                    [-P.k + P.Jp * wn_phi**2],
                    [-P.b + 2 * P.Jp * zeta_phi * wn_phi]])
        tmp = np.linalg.inv(AA) @ bb
        self.kp_phi = tmp[0][0]
        self.kd_phi = tmp[1][0]
        # DC gain for outer loop
        k_DC_phi = P.k * k_DC_th * self.kp_phi \
            / (P.k + P.k * k_DC_th * self.kp_phi)
        # print control gains to terminal        
        print('k_DC_phi', k_DC_phi)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)
        print('kp_phi: ', self.kp_phi)
        print('ki_phi:', self.ki_phi)
        print('kd_phi: ', self.kd_phi)
        #---------------------------------------------------
        # initialize variables for integrator & differentiators
        #---------------------------------------------------
        self.integrator_phi = 0.
        self.error_phi_d1 = 0.
        self.phi_dot = 0.
        self.phi_d1 = 0.
        self.theta_dot = 0.
        self.theta_d1 = 0.

    def update(self, phi_r, y):
        theta = y[0][0]
        phi = y[1][0]
        #---------------------------------------------------
        # Update Outer Loop (phi-control)
        #---------------------------------------------------
        # Compute the error in z
        error_phi = phi_r - phi
        # integrate error in phi
        self.integrator_phi = self.integrator_phi \
            + (P.Ts / 2) * (error_phi + self.error_phi_d1)
        # differentiate phi
        self.phi_dot = self.beta * self.phi_dot \
            + (1 - self.beta) * ((phi - self.phi_d1) / P.Ts)
        # PID control - unsaturated
        theta_r_unsat = self.kp_phi * error_phi \
            + self.ki_phi * self.integrator_phi \
                - self.kd_phi * self.phi_dot
        # saturate theta_r
        theta_r = saturate(theta_r_unsat, self.theta_max)
        # integrator anti - windup
        if self.ki_phi != 0.0:
            self.integrator_phi = self.integrator_phi \
                + P.Ts / self.ki_phi * (theta_r - theta_r_unsat)
        #---------------------------------------------------
        # Update Inner Loop (theta-control)
        #---------------------------------------------------
        # Compute the error in theta
        error_th = theta_r - theta
        # differentiate theta
        self.theta_dot = self.beta * self.theta_dot \
            + (1 - self.beta) * ((theta - self.theta_d1) / P.Ts)
         # PD control on theta
        tau_unsat = self.kp_th * error_th \
            - self.kd_th * self.theta_dot
        # saturate the torque
        tau = saturate(tau_unsat, P.tau_max)
        # update delayed variables
        self.error_phi_d1 = error_phi
        self.phi_d1 = phi
        self.theta_d1 = theta
        # return computed force
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u






