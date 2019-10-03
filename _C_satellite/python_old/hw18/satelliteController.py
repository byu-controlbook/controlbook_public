import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P
sys.path.append('../hw10')  # add parent directory
import satelliteParamHW10 as P10
import loopshape_in as L_in
import loopshape_out as L_out
import numpy as np


class satelliteController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.phi_dot = 0.0           # derivative of phi
        self.theta_dot = 0.0         # derivative of theta
        self.phi_d1 = 0.             # angle phi delayed by 1 sample
        self.theta_d1 = 0.0          # Angle theta delayed by 1 sample
        self.kd_phi = P10.kd_phi
        self.kd_th = P10.kd_th
        self.xout_C = np.zeros((L_out.C_ss.A.shape[0], 1))
        self.x_F = np.zeros((L_out.F_ss.A.shape[0], 1))
        self.xin_C  = np.zeros((L_in.C_ss.A.shape[0], 1))
        self.A_F = L_out.F_ss.A
        self.B_F = L_out.F_ss.B
        self.C_F = L_out.F_ss.C
        self.D_F = L_out.F_ss.D
        self.Aout_C = L_out.C_ss.A
        self.Bout_C = L_out.C_ss.B
        self.Cout_C = L_out.C_ss.C
        self.Dout_C = L_out.C_ss.D
        self.Ain_C = L_in.C_ss.A
        self.Bin_C = L_in.C_ss.B
        self.Cin_C = L_in.C_ss.C
        self.Din_C = L_in.C_ss.D
        self.limit = P.tau_max       # Maxiumum torque
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller
        self.N = 10

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        phi_r = y_r[0]
        theta = y[0]
        phi = y[1]

        # differentiate z and theta
        self.differentiatePhi(phi)
        self.differentiateTheta(theta)

        # solve differential equation defining prefilter F
        self.updatePrefilterState(phi_r)
        phi_r_filtered = self.C_F * self.x_F + self.D_F * phi_r

        # error signal for outer loop
        error_out = phi_r_filtered - phi

        # Outer loop control C_out
        self.updateControlOutState(error_out)
        theta_r = -self.kd_phi*self.phi_dot + self.Cout_C * self.xout_C + self.Dout_C * error_out

        # error signal for inner loop
        error_in = theta_r - theta

        # Inner loop control C_in
        self.updateControlInState(error_in)
        tau_unsat = -self.kd_th*self.theta_dot + self.Cin_C * self.xin_C + self.Din_C * error_in

        tau = self.saturate(tau_unsat)
        return [tau.item(0)]

    def differentiatePhi(self, phi):
        '''
            differentiate z
        '''
        self.phi_dot = self.beta*self.phi_dot + (1-self.beta)*((phi - self.phi_d1) / self.Ts)
        self.phi_d1 = phi

    def differentiateTheta(self, theta):
        '''
            differentiate theta
        '''
        self.theta_dot = self.beta*self.theta_dot + (1-self.beta)*((theta - self.theta_d1) / self.Ts)
        self.theta_d1 = theta

    def updatePrefilterState(self, r):
        for i in range(0, self.N):
            self.x_F = self.x_F + (self.Ts/self.N)*(
                self.A_F*self.x_F + self.B_F*r
            )

    def updateControlOutState(self, error_out):
        for i in range(0, self.N):
            self.xout_C = self.xout_C + (self.Ts/self.N)*(
                self.Aout_C*self.xout_C + self.Bout_C*error_out
            )

    def updateControlInState(self, error_in):
        for i in range(0, self.N):
            self.xin_C = self.xin_C + (self.Ts/self.N)*(
                self.Ain_C * self.xin_C + self.Bin_C * error_in
            )

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

