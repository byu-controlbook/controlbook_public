import numpy as np
import control as cnt
import pendulumParam as P


class ctrlStateFeedbackIntegrator:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        tr_z = 1.5        # rise time for position
        tr_theta = 0.5    # rise time for angle
        zeta_z = 0.707  # damping ratio position
        zeta_th = 0.707  # damping ratio angle
        integrator_pole = -2  # integrator pole
        # Augmented State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, -3 * P.m1 * P.g / 4 / (.25 * P.m1 + P.m2),
                -P.b / (.25 * P.m1 + P.m2), 0.0],
            [0.0, 
                3*(P.m1+P.m2) * P.g/2/(.25 * P.m1 + P.m2)/P.ell,
                3 * P.b / 2 / (.25 * P.m1 + P.m2) / P.ell, 0.0]
            ])
        B = np.array([[0.0],
                      [0.0],
                      [1 / (.25 * P.m1 + P.m2)],
                      [-3.0 / 2 / (.25 * P.m1 + P.m2) / P.ell]])
        C = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
        # form augmented system
        A1 = np.vstack((
                np.hstack((A, np.zeros((4,1)))),
                np.hstack((-Cr, np.zeros((1,1))))))
        B1 = np.vstack((B, np.zeros((1,1))))
        # gain calculation
        wn_th = 2.2 / tr_theta  # natural frequency for angle
        wn_z = 2.2 / tr_z  # natural frequency for position
        des_char_poly = np.convolve(
            np.convolve([1, 2 * zeta_z * wn_z, wn_z**2],
                        [1, 2 * zeta_th * wn_th, wn_th**2]),
            np.poly([integrator_pole]))
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            print("The system is not controllable")
        else:
            K1 = cnt.acker(A1, B1, des_poles)
            self.K = K1[0][0:4]
            self.ki = K1[0][4]
        # print gains to terminal
        print('K: ', self.K)
        print('ki: ', self.ki)
        #--------------------------------------------------
        # saturation limits
        theta_max = 30.0 * np.pi / 180.0  # Max theta, rads
        #--------------------------------------------------
        # variables to implement integrator
        self.integrator_z = 0.0  # integrator
        self.error_z_d1 = 0.0  # error signal delayed by 1 sample

    def update(self, z_r, x):
        z = x[0][0]
        # integrate error
        error_z = z_r - z
        # integrate error
        self.integrator_z = self.integrator_z \
            + (P.Ts / 2.0) * (error_z + self.error_z_d1)
        self.error_z_d1 = error_z
        # Compute the state feedback controller
        F_unsat = -self.K @ x - self.ki * self.integrator_z
        F = saturate(F_unsat[0], P.F_max)
        return F


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

