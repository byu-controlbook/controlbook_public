import numpy as np
import control as cnt
import armParam as P


class LQR:
    def __init__(self):
        #  tuning parameters
        Q = np.diag([1.0, 0.1])
        R = np.diag([1.0])

        # system model
        den = P.m * P.ell**2
        A = np.array([[0, 1],
                      [0, -3*P.b / den]])
        B = np.array([[0],
                      [3 / den]])

        # feedback gain matrix
        self.K,_,_ = cnt.lqr(A, B, Q, R)

    def _get_equilibrium_input(self, x):
        tau_eq = 0.5 * P.m * P.g * P.ell * np.cos(x[0,0])
        return tau_eq

    def update(self, x_r, x):
        x_tilde = x - x_r
        u_tilde = -self.K @ x_tilde
        u = u_tilde + self._get_equilibrium_input(x)
        u = saturate(u, P.tau_max)
        return u[0,0]


def saturate(u, limit):
    if np.abs(u) > limit:
        u = limit * np.sign(u)
    return u
