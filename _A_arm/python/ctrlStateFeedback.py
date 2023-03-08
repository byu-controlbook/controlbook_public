import numpy as np
import control as cnt
import armParam as P

class ctrlStateFeedback:
    # dirty derivatives to estimate thetadot
    def __init__(self):
        #  tuning parameters
        tr = 0.4
        zeta = 0.707
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([[0.0, 1.0],
                      [0.0, -1.0 * P.b / P.m / (P.ell**2)]])
        B = np.array([[0.0],
                      [3.0 / P.m / (P.ell**2)]])        
        C = np.array([[1.0, 0.0]])
        # gain calculation
        wn = 2.2 / tr  # natural frequency
        des_char_poly = [1, 2 * zeta * wn, wn**2]
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 2:
            print("The system is not controllable")
        else:
            self.K = (cnt.acker(A, B, des_poles))
            self.kr = -1.0 / (C @ np.linalg.inv(A - B @ self.K) @ B)
        print('K: ', self.K)
        print('kr: ', self.kr)
        print(des_poles)

    def update(self, theta_r, x):
        theta = x[0][0]
        # compute feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)
        # Compute the state feedback controller
        tau_tilde = -self.K @ x + self.kr * theta_r
        # compute total torque
        tau = saturate(tau_fl + tau_tilde[0][0], P.tau_max)
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u


