import numpy as np
import control as cnt
import armParam as P

class ctrlDisturbanceObserver :
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        #  tuning parameters
        tr = 0.4
        zeta = 0.707
        integrator_pole = 5
        wn_obs = 10            # natural frequency for observer
        zeta_obs = 0.707       # damping ratio for observer
        dist_obsv_pole = 5.5  # pole for disturbance observer

        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([[0.0, 1.0],
                      [0.0, -1.0 * P.b / P.m / (P.ell**2)]])
        B = np.array([[0.0],
                      [3.0 / P.m / (P.ell**2)]])        
        C = np.array([[1.0, 0.0]])
        # form augmented system
        A1 = np.array([[0.0, 1.0, 0.0],
                       [0.0, -1.0 * P.b / P.m / (P.ell**2), 0.0],
                       [-1.0, 0.0, 0.0]])
        B1 = np.array([[0.0],
                       [3.0 / P.m / (P.ell**2)],
                       [0.0]])

        # gain calculation
        wn = 2.2 / tr  # natural frequency
        #wn = 0.5*np.pi/(tr*np.sqrt(1-zeta**2)) # natural frequency
        des_char_poly = np.convolve([1, 2 * zeta * wn, wn**2],
                                    [1, integrator_pole])
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
            print("The system is not controllable")
        else:
            K1 = cnt.place(A1, B1, des_poles)
            self.K = K1[0][0:2]
            self.ki = K1[0][2]
        # observer design
        # Augmented Matrices
        A2 = np.concatenate((
                            np.concatenate((A, B), axis=1),
                            np.zeros((1, 3))),
                        axis=0)
        B2 = np.concatenate((B, np.zeros((1, 1))), axis=0)
        C2 = np.concatenate((C, np.zeros((1, 1))), axis=1)
        des_char_est = np.array([1., 2.*zeta*wn_obs, wn_obs**2.])
        des_obsv_char_poly = np.convolve([1, 2 * zeta_obs * wn_obs, wn_obs**2],
                                         [1, dist_obsv_pole])
        des_obsv_poles = np.roots(des_obsv_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A2.T, C2.T)) != 3:
            print("The system is not observerable")
        else:
            L2 = cnt.acker(A2.T, C2.T, des_obsv_poles).T
        print('K: ', self.K)
        print('ki ', self.ki)
        print('L^T: ', L2.T)
        #--------------------------------------------------
        # variables to implement integrator
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.obsv_state = np.array([
            [0.0],  # theta_hat_0
            [0.0],  # thetadot_hat_0
            [0.0],  # estimate of the disturbance
        ])
        self.tau_d1 = 0.0  # control torque, delayed 1 sample
        self.L = L2
        self.A = A2
        self.B = B1
        self.C = C2
        self.tau_d1 = 0.0      # control torque, delayed 1 sample

    def update(self, theta_r, y_m):
        # update the observer and extract theta_hat
        x_hat, d_hat = self.update_observer(y_m)
        theta_hat = x_hat[0][0]
        # integrate error
        error = theta_r - theta_hat
        self.integrator = self.integrator + (P.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error
        # compute feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)
        # Compute the state feedback controller
        tau_tilde = -self.K @ x_hat - self.ki * self.integrator - d_hat

        # compute total torque
        tau = saturate(tau_fl + tau_tilde[0], P.tau_max)
        self.tau_d1 = tau
        return tau, x_hat, d_hat

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.obsv_state, y_m)
        F2 = self.observer_f(self.obsv_state + P.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.obsv_state + P.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.obsv_state + P.Ts * F3, y_m)
        self.obsv_state += P.Ts / 6 *  (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = self.obsv_state[0:2]
        d_hat = self.obsv_state[2][0]
        return x_hat, d_hat

    def observer_f(self, x_hat, y_m):
        # compute feedback linearizing torque tau_fl
        theta_hat = x_hat[0][0]
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)
        # xhatdot = A*xhat + B*(u-ue) + L(y-C*xhat)
        xhat_dot = self.A @ x_hat\
                   + self.B * (self.tau_d1 - tau_fl)\
                   + self.L * (y_m - self.C @ x_hat)
        return xhat_dot


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u


