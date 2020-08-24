import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P
import loopshape_in as L_in
import loopshape_out as L_out
import numpy as np

class pendulumController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.xout_C = np.zeros((L_out.C_ss.A.shape[0], 1))
        self.xout_F = np.zeros((L_out.F_ss.A.shape[0], 1))
        self.xin_C  = np.zeros((L_in.C_ss.A.shape[0], 1))
        self.Aout_F = L_out.F_ss.A
        self.Bout_F = L_out.F_ss.B
        self.Cout_F = L_out.F_ss.C
        self.Dout_F = L_out.F_ss.D
        self.Aout_C = L_out.C_ss.A
        self.Bout_C = L_out.C_ss.B
        self.Cout_C = L_out.C_ss.C
        self.Dout_C = L_out.C_ss.D
        self.Ain_C = L_in.C_ss.A
        self.Bin_C = L_in.C_ss.B
        self.Cin_C = L_in.C_ss.C
        self.Din_C = L_in.C_ss.D
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller
        self.N = 10  #number of Euler integration steps for each sample

    def update(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r
        z = y[0]
        theta = y[1]

        # solve differential equation defining prefilter F
        self.updatePrefilterState(z_r)
        z_r_filtered = self.Cout_F * self.xout_F + self.Dout_F * z_r

        # error signal for outer loop
        error_out = z_r_filtered - z

        # Outer loop control C_out
        self.updateControlOutState(error_out)
        theta_r = self.Cout_C * self.xout_C + self.Dout_C * error_out

        # error signal for inner loop
        error_in = theta_r - theta

        # Inner loop control C_in
        self.updateControlInState(error_in)
        F_unsat = self.Cin_C * self.xin_C + self.Din_C * error_in

        F_sat = self.saturate(F_unsat)
        return [F_sat.item(0)]

    def updatePrefilterState(self, z_r):
        for i in range(0, self.N):
            self.xout_F = self.xout_F + (self.Ts/self.N)*(
                self.Aout_F*self.xout_F + self.Bout_F*z_r
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

