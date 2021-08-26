import sys
sys.path.append('..')  # add parent directory
import armParam as P
import loopshape_arm as L
import numpy as np
from discreteFilter import discreteFilter

class armController:
    def __init__(self, method="state_space"):
        if method == "state_space":
            self.x_C = np.zeros((L.C_ss.A.shape[0], 1))
            self.x_F = np.zeros((L.F_ss.A.shape[0], 1))
            self.A_C = L.C_ss.A
            self.B_C = L.C_ss.B
            self.C_C = L.C_ss.C
            self.D_C = L.C_ss.D
            self.A_F = L.F_ss.A
            self.B_F = L.F_ss.B
            self.C_F = L.F_ss.C
            self.D_F = L.F_ss.D
            self.N = 10  #number of Euler integration steps for each sample

        elif method == "digital_filter":
            self.prefilter = discreteFilter(L.F.num, L.F.den, P.Ts)
            self.control = discreteFilter(L.C.num, L.C.den, P.Ts)

        self.limit = P.tau_max  # Maximum torque
        self.Ts = P.Ts  # sample rate of
        self.method = method


    def update(self, theta_r, y):
        theta = y.item(0)

        #prefilter
        if self.method == "state_space":
            self.updatePrefilterState(theta_r)
            theta_r_filtered = self.C_F @ self.x_F + self.D_F * theta_r
        elif self.method == "digital_filter":
            theta_r_filtered = self.prefilter.update(theta_r)


        # filtered error signal
        error = theta_r_filtered - theta

        # update controller
        if self.method == "state_space":
            self.updateControlState(error)
            tau_tilde = (self.C_C @ self.x_C + self.D_C * error)[0,0]
        elif self.method == "digital_filter":
            tau_tilde = self.control.update(error)

        # compute equilibrium torque tau_e
        tau_e = P.m * P.g * (P.ell / 2.0) * np.cos(theta)

        # compute total torque
        tau = self.saturate(tau_e + tau_tilde)
        return tau


    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

    def updatePrefilterState(self, theta_r):
        for i in range(0, self.N):
            self.x_F = self.x_F + (self.Ts/self.N)*(
                self.A_F*self.x_F + self.B_F*theta_r
            )

    def updateControlState(self, error):
        for i in range(0, self.N):
            self.x_C = self.x_C + (self.Ts/self.N)*(
                self.A_C*self.x_C + self.B_C*error
            )
