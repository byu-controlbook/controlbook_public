import sys
sys.path.append('..')  # add parent directory
import armParam as P
import loopshape_arm as L
import numpy as np
from transferFunction import transferFunction

class armController:
    # state feedback control using dirty derivatives to estimate thetadot
    def __init__(self):
        self.x_C = np.zeros((L.C_ss.A.shape[0], 1))
        self.x_F = np.zeros((L.F_ss.A.shape[0], 1))
        self.A_F = L.F_ss.A
        self.B_F = L.F_ss.B
        self.C_F = L.F_ss.C
        self.D_F = L.F_ss.D
        self.A_C = L.C_ss.A
        self.B_C = L.C_ss.B
        self.C_C = L.C_ss.C
        self.D_C = L.C_ss.D
        self.limit = P.tau_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller
        self.N = 10  # number of Euler integration steps for each sample

    def update(self, theta_r, theta):

         # solve differential equation defining prefilter F
        self.updatePrefilterState(theta_r)
        theta_r_filtered = self.C_F * self.x_F + self.D_F * theta_r

        # filtered error signal
        error = theta_r_filtered - theta

        # solve differential equation defining control C
        self.updateControlState(error)
        tau_tilde = self.C_C * self.x_C + self.D_C * error

        # compute equilibrium torque tau_e
        tau_e = P.m * P.g * (P.ell / 2.0) * np.cos(theta)

        # compute total torque
        tau = self.saturate(tau_e + tau_tilde)
        return [tau.item(0)]

    def updatePrefilterState(self, z_r):
        for i in range(0, self.N):
            self.x_F = self.x_F + (self.Ts/self.N)*(
                self.A_F*self.x_F + self.B_F*z_r
            )

    def updateControlState(self, error):
        for i in range(0, self.N):
            self.x_C = self.x_C + (self.Ts/self.N)*(
                self.A_C*self.x_C + self.B_C*error
            )

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
