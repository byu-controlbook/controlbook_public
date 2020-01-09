import numpy as np
import pendulumParamHW14 as P

class pendulumController:
    def __init__(self):
        self.obsv_state = np.array([
            [0.0],  # initial estimate for z_hat
            [0.0],  # initial estimate for theta_hat
            [0.0],  # initial estimate for z_hat_dot
            [0.0],   # initial estimate for theta_hat_dot
            [0.0],  # estimate of the disturbance
        ])
        self.F_d1 = 0.0  # Computed Force, delayed by one sample
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.ki = P.ki  # integrator gain
        self.L = P.L2  # observer gain
        self.A = P.A2  # system model
        self.B = P.B1
        self.C = P.C2
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, z_r, y):
        # update the observer and extract z_hat
        x_hat, d_hat = self.update_observer(y)
        z_hat = x_hat.item(0)

        # integrate error
        error = z_r - z_hat
        self.integrateError(error)

        # Compute the state feedback controller
        F_unsat = -self.K @ x_hat \
                  - self.ki * self.integrator \
                  - d_hat
        F = self.saturate(F_unsat.item(0))
        self.F_d1 = F
        return F, x_hat, d_hat

    def update_observer(self, y):
        # update the observer using RK4 integration
        F1 = self.obsv_f(self.obsv_state, y)
        F2 = self.obsv_f(self.obsv_state + self.Ts / 2 * F1, y)
        F3 = self.obsv_f(self.obsv_state + self.Ts / 2 * F2, y)
        F4 = self.obsv_f(self.obsv_state + self.Ts * F3, y)
        self.obsv_state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = np.array([[self.obsv_state.item(0)],
                          [self.obsv_state.item(1)],
                          [self.obsv_state.item(2)],
                          [self.obsv_state.item(3)]])
        d_hat = self.obsv_state.item(4)
        return x_hat, d_hat

    def obsv_f(self, x_hat, y_m):
        # xhatdot = A*xhat + B*(u-ue) + L(y-C*xhat)
        xhat_dot = self.A @ x_hat \
                   + self.B * self.F_d1 \
                   + self.L @ (y_m - self.C @ x_hat)
        return xhat_dot

    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

