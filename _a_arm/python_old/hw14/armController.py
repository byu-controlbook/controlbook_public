import numpy as np
import armParamHW14 as P

class armController:
    # state feedback control using dirty derivatives to estimate thetadot
    def __init__(self):
        self.x_hat = np.matrix([
            [0.0],
            [0.0],
        ])
        self.d_hat = 0.0             # initial estimate for disturbance
        self.tau_d1 = 0.0            # control torque, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # Input gain
        self.L = P.L                 # observer gain
        self.Ld = P.Ld
        self.A = P.A                 # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.tau_max         # Maxiumum force
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y_m):
        # y_r is the referenced input
        # y is the current state
        theta_r = y_r[0]

        # update the observer and extract theta_hat
        self.updateObserver(y_m)
        theta_hat = self.x_hat[0]

        # integrate error
        error = theta_r - theta_hat
        self.integrateError(error)

        # compute equilibrium torque tau_e
        theta_hat = self.x_hat[0]
        tau_e = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)

        # Compute the state feedback controller
        tau_tilde = -self.K*self.x_hat - self.ki*self.integrator

        # compute total torque
        tau = self.saturate(tau_e + tau_tilde)
        self.updateTorque(tau)
        return [tau.item(0)]

    def updateObserver(self, y_m):
        # compute equilibrium torque tau_e
        theta_e = 0.0
        theta_hat = self.x_hat[0]
        tau_e = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)
        x_e = np.matrix([
            [theta_e],
            [0.0],
        ])

        N = 10
        for i in range(0, N):
            self.x_hat = self.x_hat + self.Ts/float(N)*(
                self.A*(self.x_hat - x_e)
                + self.B*(self.tau_d1 - tau_e + self.d_hat)
                + self.L*(y_m-self.C*self.x_hat)
            )
        self.d_hat = self.d_hat + self.Ts/float(N)*(
                self.Ld*(y_m - self.C*self.x_hat)
            )

    def updateTorque(self, tau):
        self.tau_d1 = tau

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

