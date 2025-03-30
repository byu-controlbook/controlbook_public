import numpy as np
import control as cnt
import satelliteParam as P


class LQR:
    def __init__(self):
        #  tuning parameters
        Q = np.diag([0.1, 1.0, 0.9, 4.5])
        R = np.diag([0.0001])

        # system model
        k_Js, k_Jp = P.k / P.Js, P.k / P.Jp
        b_Js, b_Jp = P.b / P.Js, P.b / P.Jp
        A = np.array([[0, 0, 1, 0],
                      [0, 0, 0, 1],
                      [-k_Js, k_Js, -b_Js, b_Js],
                      [k_Jp, -k_Jp, b_Jp, -b_Jp]])
        B = np.array([[0],
                      [0],
                      [1 / P.Js],
                      [0]])

        # feedback gain matrix
        self.K,_,_ = cnt.lqr(A, B, Q, R)

    def update(self, x_r, x):
        x_error = x - x_r
        u = -self.K @ x_error
        u = saturate(u, P.tau_max)
        return u[0,0]


def saturate(u, limit):
    if np.abs(u) > limit:
        u = limit * np.sign(u)
    return u
