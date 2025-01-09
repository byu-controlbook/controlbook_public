import numpy as np
import control as cnt
import armParam as P


class KalmanFilter:
    def __init__(self):
        #  tuning parameters
        self.cov = np.eye(2) # covariance of state estimate
        self.Q = np.diag([0.001, 0.001]) # process noise (small because dynamics doesn't have any)
        self.R = np.diag([0.01]) # measurement noise (small because sensor model doesn't have any)

        # system model
        den = P.m * P.ell**2
        self.A = np.array([[0, 1],
                           [0, -3*P.b / den]])
        self.B = np.array([[0],
                           [3 / den]])
        self.G = np.eye(2)
        self.C = np.array([[1, 0]])

        self.u_prev = 0.0
        self.x_hat = np.zeros((2,1))
        self.error_prev = 0.0

        # Kalman gain using control module only needs to be calculated once
        # because it doesn't depend on the covariance of the state estimate
        # NOTE: Not needed if using method from Probabilistic Robotics
        self.K,_,_ = cnt.lqe(self.A, self.G, self.C, self.Q, self.R)

    def update(self, y, u):
        u = np.array([[u]])
        x_hat, cov = self._prediction_step(self.x_hat, u)
        self.x_hat, self.cov = self._correction_step(x_hat, cov, y)
        return self.x_hat.copy() # return a copy to prevent unwanted mutation

    def _prediction_step(self, x_hat, u):
        x_hat = self._rk4_step(x_hat, u)

        At = self.A * P.Ts
        Ad = np.eye(2) + At + 0.5*(At @ At) # first 3 terms from matrix exponential
        sigma = Ad @ self.cov @ Ad.T + self.Q
        return x_hat, sigma

    def _correction_step(self, x_hat, sigma, y):
        ## Chose a method to calculate Kalman gain: both work for this example
        ## from Probabilistic Robotics, Thrun et al., Algorithm 3.1
        K = sigma @ self.C.T @ np.linalg.inv(self.C @ sigma @ self.C.T + self.R)
        ## from control module (calculated once in constructor)
        # K = self.K

        x_hat = x_hat + K @ (y - self.C @ x_hat)
        sigma = (np.eye(2) - K @ self.C) @ sigma
        return x_hat, sigma

    def _get_equilibrium_input(self, x):
        tau_eq = 0.5 * P.m * P.g * P.ell * np.cos(x[0,0])
        return tau_eq

    def _rk4_step(self, xhat, u):
        f1 = self._f(xhat, u)
        f2 = self._f(xhat + f1*P.Ts/2, u)
        f3 = self._f(xhat + f2*P.Ts/2, u)
        f4 = self._f(xhat + f3*P.Ts, u)
        return xhat + (f1 + 2*f2 + 2*f3 + f4) * P.Ts/6

    def _f(self, x_hat, u):
        x_eq = x_hat * np.array([[1,0]]).T # ensure velocity is zero for equilibrium
        x_tilde = x_hat - x_eq
        # x_tilde = x_hat # unsure if x_hat is tilde variable already or not
        u_eq = self._get_equilibrium_input(x_hat)
        u_tilde = u - u_eq
        xhat_dot = self.A @ x_tilde + self.B @ u_tilde
        return xhat_dot

    ## TODO: clean up - decide whether to include correction in dynamics (code below)
    ## or have a separate correction step to already propagated mean (code above)
    ## NOTE: the code below is not currently working

    # def _rk4_step(self, xhat, y, u):
    #     f1 = self._f(xhat, y, u)
    #     f2 = self._f(xhat + f1*P.Ts/2, y, u)
    #     f3 = self._f(xhat + f2*P.Ts/2, y, u)
    #     f4 = self._f(xhat + f3*P.Ts, y, u)
    #     return xhat + (f1 + 2*f2 + 2*f3 + f4) * P.Ts/6

    # def _f(self, x_hat, y, u):
    #     x_eq = x_hat * np.array([[1,0]]).T
    #     x_tilde = x_hat - x_eq
    #     x_tilde = x_hat
    #     u_eq = self._get_equilibrium_input(x_hat)
    #     u_tilde = u - u_eq
    #     y_tilde = y - self.C @ x_hat
    #     xhat_dot = self.A @ x_tilde + self.B @ u_tilde + self.K @ y_tilde
    #     return xhat_dot
