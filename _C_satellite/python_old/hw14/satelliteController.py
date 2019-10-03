import numpy as np
import satelliteParamHW14 as P

class satelliteController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.x_hat = np.matrix([
            [0.0],                   # initial estimate for theta_hat
            [0.0],                   # initial estimate for phi_hat
            [0.0],                   # initial estimate for theta_hat_dot
            [0.0]])                  # initial estimate for phi_hat_dot
        self.d_hat = 0.0             # estimate of the disturbance
        self.tau_d1 = 0.0            # Computed torque, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # integrator gain
        self.L = P.L                 # observer gain
        self.Ld = P.Ld               # gain for disturbance observer
        self.A = P.A                 # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.tau_max       # Maxiumum torque
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        phi_r = y_r[0]

        # update the observer and extract z_hat
        self.updateObserver(y)
        phi_hat = self.x_hat[1]

        # integrate error
        error = phi_r - phi_hat
        self.integrateError(error)

        # Compute the state feedback controller
        tau_unsat = -self.K*self.x_hat - self.ki*self.integrator - self.d_hat

        tau = self.saturate(tau_unsat)
        self.updateTorque(tau)
        return [tau.item(0)]

    def updateObserver(self, y_m):
        N = 10
        y = np.matrix([
            [y_m[0]],
            [y_m[1]]])
        for i in range(0, N):
            self.x_hat = self.x_hat + self.Ts/float(N)*(
                self.A*self.x_hat
                + self.B*(self.tau_d1 +self.d_hat)
                + self.L*(y-self.C*self.x_hat)
            )
            self.d_hat = self.d_hat + self.Ts/float(N)*(
                self.Ld*(y-self.C*self.x_hat)
            )

    def updateTorque(self, tau):
        self.tau_d1 = tau

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

