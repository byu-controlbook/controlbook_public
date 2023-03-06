import pendulumParam as P
import numpy as np
from scipy import signal
import control as cnt


class ctrlDisturbanceObserver:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        tr_z = 1.5        # rise time for position
        tr_theta = 0.5    # rise time for angle
        zeta_z = 0.707  # damping ratio position
        zeta_th = 0.707  # damping ratio angle
        integrator_pole = -10.0  #integrator pole
        tr_z_obs = tr_z/10.0          # rise time for observer - position
        tr_theta_obs = tr_theta / 10.0  # rise time for observer - angle
        dist_obsv_pole = -1.0  # pole for disturbance observer
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, -3 * P.m1 * P.g / 4 / (.25 * P.m1 + P.m2),
                -P.b / (.25 * P.m1 + P.m2), 0.0],
            [0.0, 
             3 * (P.m1 + P.m2)*P.g/2/(0.25*P.m1 + P.m2) / P.ell,
            3 * P.b / 2 / (.25 * P.m1 + P.m2) / P.ell, 0.0]])
        B = np.array([[0.0],
                      [0.0],
                      [1 / (.25 * P.m1 + P.m2)],
                      [-3.0 / 2 / (.25 * P.m1 + P.m2) / P.ell]])
        C = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        # form augmented system
        Cr = np.array([[1, 0]]) @ C
        A1 = np.concatenate((
                np.concatenate((A, np.zeros((4, 1))), axis=1),
                np.concatenate((-Cr, np.matrix([[0.0]])), axis=1)),
                axis=0)
        B1 = np.concatenate((B, np.matrix([[0.0]])), axis=0)
        # control gain calculation
        wn_th = 2.2 / tr_theta  # natural frequency for angle
        wn_z = 2.2 / tr_z  # natural frequency for position
        des_char_poly = np.convolve(
                np.convolve([1, 2 * zeta_z * wn_z, wn_z**2],
                            [1, 2 * zeta_th * wn_th, wn_th**2]),
                np.poly([integrator_pole]))
        des_poles = np.roots(des_char_poly)
        # Compute the control gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            print("The system is not controllable")
        else:
            K1 = cnt.acker(A1, B1, des_poles)
            self.K = K1[0][0:4]
            self.ki = K1[0][4]
        # compute observer gains
        # Augmented Matrices
        A2 = np.concatenate((
                            np.concatenate((A, B), axis=1),
                            np.zeros((1, 5))),
                            axis=0)
        C2 = np.concatenate((C, np.zeros((2, 1))), axis=1)
        wn_z_obs = 2.2 / tr_z_obs
        wn_th_obs = 2.2 / tr_theta_obs
        des_obs_char_poly = np.convolve(
                np.convolve([1, 2 * zeta_z * wn_z_obs, wn_z_obs**2],
                            [1, 2*zeta_th*wn_th_obs, wn_th_obs**2]),
                np.poly([dist_obsv_pole]))
        des_obs_poles = np.roots(des_obs_char_poly)
        # Compute the observer gains if the system is observable
        if np.linalg.matrix_rank(cnt.ctrb(A2.T, C2.T)) != 5:
            print("The system is not observable")
        else:
            L2 = signal.place_poles(A2.T, C2.T, des_obs_poles).gain_matrix.T
        # print gains to terminal
        print('K: ', self.K)
        print('ki: ', self.ki)
        print('L^T: ', L2.T)
        #--------------------------------------------------
        # saturation limits
        theta_max = 30.0 * np.pi / 180.0  # Max theta, rads
        #--------------------------------------------------
        # variables to implement integrator
        self.integrator_z = 0.0  # integrator
        self.error_z_d1 = 0.0  # error signal delayed by 1 sample
        # estimated state variables
        self.obsv_state = np.array([
            [0.0],  # initial estimate for z_hat
            [0.0],  # initial estimate for theta_hat
            [0.0],  # initial estimate for z_hat_dot
            [0.0],   # initial estimate for theta_hat_dot
            [0.0],  # estimate of the disturbance
        ])
        self.F_d1 = 0.0  # Computed Force, delayed by one sample
        self.L = L2  # observer gain
        self.A = A2  # system model
        self.B = B1
        self.C = C2

    def update(self, z_r, y):
        # update the observer and extract z_hat
        x_hat, d_hat = self.update_observer(y)
        z_hat = x_hat[0][0]
        # integrate error
        error_z = z_r - z_hat
        self.integrator_z = self.integrator_z \
            + (P.Ts / 2.0) * (error_z + self.error_z_d1)
        self.error_z_d1 = error_z
        # Compute the observer based controller
        F_unsat = -self.K @ x_hat - self.ki * self.integrator_z - d_hat
        F = saturate(F_unsat, P.F_max)
        self.F_d1 = F
        return F[0], x_hat, d_hat

    def update_observer(self, y):
        # update the observer using RK4 integration
        F1 = self.obsv_f(self.obsv_state, y)
        F2 = self.obsv_f(self.obsv_state + P.Ts / 2 * F1, y)
        F3 = self.obsv_f(self.obsv_state + P.Ts / 2 * F2, y)
        F4 = self.obsv_f(self.obsv_state + P.Ts * F3, y)
        self.obsv_state += P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = self.obsv_state[0:4]
        d_hat = self.obsv_state[4][0]
        return x_hat, d_hat

    def obsv_f(self, x_hat, y_m):
        # xhatdot = A*xhat + B*(u-ue) + L(y-C*xhat)
        xhat_dot = self.A @ x_hat \
                   + self.B * self.F_d1 \
                   + self.L @ (y_m - self.C @ x_hat)
        return xhat_dot


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

