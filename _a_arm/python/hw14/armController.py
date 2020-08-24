import numpy as np
import armParamHW14 as P

class armController:
    def __init__(self):
        self.observer_state = np.array([
            [0.0],  # estimate of theta
            [0.0],  # estimate of theta_hat
            [0.0],  # estimate of disturbance
        ])
        self.tau_d1 = 0.0      # control torque, delayed 1 sample
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0    # error signal delayed by 1 sample
        self.K = P.K           # state feedback gain
        self.ki = P.ki         # Input gain
        self.L = P.L           # observer gain
        self.Ld = P.Ld
        self.L2 = P.L2
        self.A2 = P.A2         # system model
        self.B2 = P.B2
        self.C2 = P.C2
        self.limit = P.tau_max # Maxiumum force
        self.Ts = P.Ts         # sample rate of controller

    def update(self, theta_r, y_m):
        # update the observer and extract theta_hat
        x_hat, d_hat = self.update_observer(y_m)
        theta_hat = x_hat.item(0)

        # integrate error
        error = theta_r - theta_hat
        self.integrateError(error)

        # compute feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)

        # Compute the state feedback controller
        tau_tilde = -self.K @ x_hat \
                            - self.ki * self.integrator - d_hat

        # compute total torque
        tau = self.saturate(tau_fl + tau_tilde.item(0))
        self.tau_d1 = tau

        return tau, x_hat, d_hat

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.observer_state, y_m)
        F2 = self.observer_f(self.observer_state \
                             + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.observer_state \
                             + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.observer_state \
                             + self.Ts * F3, y_m)
        self.observer_state += self.Ts / 6 * \
                               (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = np.array([[self.observer_state.item(0)],
                          [self.observer_state.item(1)]])
        d_hat = self.observer_state.item(2)

        return x_hat, d_hat

    def observer_f(self, x_hat, y_m):
        # compute feedback linearizing torque tau_fl
        theta_hat = x_hat.item(0)
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)
        # xhatdot = A*xhat + B*(u-ue) + L(y-C*xhat)
        xhat_dot = self.A2 @ x_hat\
                   + self.B2 * (self.tau_d1 - tau_fl)\
                   + self.L2 * (y_m - self.C2 @ x_hat)

        return xhat_dot

    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

