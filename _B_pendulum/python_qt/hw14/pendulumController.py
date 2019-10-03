import numpy as np
import pendulumParamHW14 as P

class pendulumController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.x_hat = np.matrix([
            [0.0],                   # initial estimate for z_hat
            [0.0],                   # initial estimate for theta_hat
            [0.0],                   # initial estimate for z_hat_dot
            [0.0]])                  # initial estimate for theta_hat_dot
        self.d_hat = 0.0             # estimate of the disturbance
        self.F_d1 = 0.0              # Computed Force, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # integrator gain
        self.L = P.L                 # observer gain
        self.Ld = P.Ld               # gain for disturbance observer
        self.A = P.A                 # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.F_max         # Maximum force
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]

        # update the observer and extract z_hat
        self.updateObserver(y)
        z_hat = self.x_hat[0]

        # integrate error
        error = z_r - z_hat
        self.integrateError(error)

        # Compute the state feedback controller
        F_unsat = -self.K*self.x_hat - self.ki*self.integrator - self.d_hat

        F_sat = self.saturate(F_unsat)
        self.updateForce(F_sat)
        return [F_sat.item(0)]

    def updateObserver(self, y_m):
        N = 10
        y = np.matrix([
            [y_m[0]],
            [y_m[1]]])
        for i in range(0, N):
            self.x_hat = self.x_hat + self.Ts/float(N)*(
                self.A*self.x_hat + self.B*self.F_d1 + self.L*(y-self.C*self.x_hat)
            )
            self.d_hat = self.d_hat + self.Ts/float(N)*(
                self.Ld*(y-self.C*self.x_hat)
            )

    def updateForce(self, F):
        self.F_d1 = F

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

