import numpy as np
import satelliteParamHW13 as P

class satelliteController:
    def __init__(self):
        self.x_hat = np.array([
            [0.0],  # initial estimate for theta_hat
            [0.0],  # initial estimate for phi_hat
            [0.0],  # initial estimate for theta_hat_dot
            [0.0],  # initial estimate for phi_hat_dot
        ])
        self.tau_d1 = 0.0              # Computed torque, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # integrator gain
        self.L = P.L                 # observer gain
        self.A = P.A                 # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.tau_max       # Maxiumum torque
        self.Ts = P.Ts               # sample rate of controller

    def update(self, phi_r, y):
        # update the observer and extract z_hat
        x_hat = self.update_observer(y)
        phi_hat = x_hat.item(1)

        # integrate error
        error = phi_r - phi_hat
        self.integrateError(error)

        # Compute the state feedback controller
        tau_unsat = -self.K @ x_hat \
                    - self.ki*self.integrator
        tau = self.saturate(tau_unsat.item(0))
        self.tau_d1 = tau
        return tau, x_hat

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y_m)
        F2 = self.observer_f(self.x_hat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.x_hat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.x_hat + self.Ts * F3, y_m)
        self.x_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = self.A @ x_hat \
                   + self.B * self.tau_d1 \
                   + self.L @ (y_m - self.C @ x_hat)
        return xhat_dot

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

