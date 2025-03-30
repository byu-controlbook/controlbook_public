import numpy as np
import control as cnt
import pendulumParam as P


class LQR:
    def __init__(self):
        #  tuning parameters
        Q = np.diag([1.0, 1.0, 1.0, 1.0])
        R = np.diag([1.0])

        # system model
        zd_den = .25*P.m1 + P.m2
        thd_den = 2 * zd_den * P.ell
        A = np.array([[0, 0, 1, 0],
                      [0, 0, 0, 1],
                      [0, -0.75*P.m1*P.g / zd_den, -P.b / zd_den, 0],
                      [0, 3*(P.m1 + P.m2)*P.g / thd_den, 3*P.b / thd_den, 0]])

        B = np.array([[0.0],
                      [0.0],
                      [1 / zd_den],
                      [-3 / thd_den]])

        # feedback gain matrix
        self.K,_,_ = cnt.lqr(A, B, Q, R)

    def update(self, x_r, x):
        x_tilde = x - x_r
        u_tilde = -self.K @ x_tilde
        u = u_tilde + self._get_equilibrium_input(x)
        u = saturate(u, P.F_max)
        return u[0,0]

    def _get_equilibrium_input(self, x):
        force_eq = 0.0
        return force_eq


def saturate(u, limit):
    if np.abs(u) > limit:
        u = limit * np.sign(u)
    return u
