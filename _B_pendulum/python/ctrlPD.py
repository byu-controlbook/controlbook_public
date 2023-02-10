import numpy as np
import pendulumParam as P

class ctrlPD:
    def __init__(self):
        ####################################################
        #       PD Control: Time Design Strategy
        ####################################################
        # tuning parameters
        tr_th = 0.15          # Rise time for inner loop (theta)
        zeta_th = 0.707       # inner loop Damping Coefficient
        M = 15.0              # Time scale separation 
        zeta_z = 0.707        # outer loop Damping Coefficient
        # saturation limits
        F_max = 5             		  # Max Force, N
        error_max = 1        		  # Max step size,m
        theta_max = 30.0 * np.pi / 180.0  # Max theta, rads
        #---------------------------------------------------
        #                    Inner Loop
        #---------------------------------------------------
        # parameters of the open loop transfer function
        b0_th = 1.0 / (P.m1 * (P.ell / 6.0) \
            + P.m2 * (2.0 * P.ell / 3.0))
        a0_th = (P.m1 + P.m2) * P.g / (P.m1 * (P.ell / 6.0) \
            + P.m2 * (2.0 * P.ell / 3.0))
        # coefficients for desired inner loop
        wn_th = 2.2 / tr_th     # Natural frequency
        # compute gains
        self.kp_th = -(wn_th**2 + a0_th) / b0_th
        self.kd_th = -(2.0 * zeta_th * wn_th) / b0_th
        DC_gain = b0_th * self.kp_th / (b0_th * self.kp_th + a0_th)
        #---------------------------------------------------
        #                    Outer Loop
        #---------------------------------------------------
        # coefficients for desired outer loop
        tr_z = M * tr_th  # desired rise time, s
        wn_z = 2.2 / tr_z  # desired natural frequency
        # compute gains
        a = wn_z**2*np.sqrt(2.0*P.ell/3.0/P.g)-2.0*zeta_z*wn_z
        self.kd_z = a / (a + np.sqrt(3.0*P.g/2.0/P.ell))
        self.kp_z = -wn_z**2*np.sqrt(2.0*P.ell/3.0/P.g)*(1+self.kd_z)
        # print control gains to terminal        
        print('DC_gain', DC_gain)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)
        print('kp_z: ', self.kp_z)
        print('kd_z: ', self.kd_z)
        #---------------------------------------------------
        #                    zero canceling filter
        #---------------------------------------------------
        self.filter = zeroCancelingFilter(DC_gain)

    def update(self, z_r, state):
        z = state[0][0]
        theta = state[1][0]
        zdot = state[2][0]
        thetadot = state[3][0]
        # the reference angle for theta comes from the
        # outer loop PD control
        tmp = self.kp_z * (z_r - z) - self.kd_z * zdot
        # low pass filter the outer loop to cancel
        # left-half plane zero and DC-gain
        theta_r = self.filter.update(tmp)
        # the force applied to the cart comes from the
        # inner loop PD control
        F = self.kp_th * (theta_r - theta) - self.kd_th * thetadot
        return F

class zeroCancelingFilter:
    def __init__(self, DC_gain):
        self.a = -3.0 / (2.0 * P.ell * DC_gain)
        self.b = np.sqrt(3.0 * P.g / (2.0 * P.ell))
        self.state = 0.0

    def update(self, input):
        # integrate using RK1
        self.state += P.Ts * (-self.b * self.state + self.a * input)
        return self.state






