import numpy as np
import armParamHW13 as P

class armController:
    # state feedback control using dirty derivatives to estimate thetadot
    def __init__(self):
        self.x_hat = np.array([
            [0.0],  # theta_hat_0
            [0.0],  # thetadot_hat_0
        ])
        self.tau_d1 = 0.0  # control torque, delayed by one sample
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.ki = P.ki  # Input gain
        self.L = P.L  # observer gain
        self.A = P.A  # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.tau_max  # Maxiumum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, theta_r, y):
        # update the observer and extract theta_hat
        x_hat = self.update_observer(y)
        theta_hat = x_hat.item(0)

        # integrate error
        error = theta_r - theta_hat
        self.integrateError(error)

        # feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)

        # Compute the state feedback controller
        tau_tilde = -self.K @ x_hat - self.ki * self.integrator

        # compute total torque
        tau = self.saturate(tau_fl + tau_tilde.item(0))
        self.tau_d1 = tau
        return tau, x_hat

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y_m)
        F2 = self.observer_f(self.x_hat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.x_hat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.x_hat + self.Ts * F3, y_m)
        self.x_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y_m):
        # compute feedback linearizing torque tau_fl
        theta_hat = x_hat.item(0)
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)
        # xhatdot = A*xhat + B*(u-ue) + L(y-C*xhat)
        xhat_dot = self.A @ x_hat\
                   + self.B * (self.tau_d1 - tau_fl)\
                   + self.L * (y_m - self.C @ x_hat)
        return xhat_dot

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

