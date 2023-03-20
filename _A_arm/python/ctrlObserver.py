import numpy as np
import control as cnt
import armParam as P


class ctrlObserver:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        #  tuning parameters
        tr = 0.4
        zeta = 0.707
        integrator_pole = 9
        tr_obs = tr/10  # rise time frequency for observer
        zeta_obs = 0.707  # damping ratio for observer
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        self.A = np.array([[0.0, 1.0],
                          [0.0, -1.0 * P.b / P.m / (P.ell**2)]])
        self.B = np.array([[0.0],
                           [3.0 / P.m / (P.ell**2)]])        
        self.C = np.array([[1.0, 0.0]])
        # form augmented system
        A1 = np.vstack((np.hstack((self.A, np.zeros((2,1)))), 
                        np.hstack((-self.C, np.zeros((1,1)))) ))
        B1 = np.vstack( (self.B, 0.0) )
        # gain calculation
        wn = 2.2 / tr  # natural frequency
        des_char_poly = np.convolve([1, 2*zeta*wn, wn**2],
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
        wn_obs = 2.2 / tr_obs
        des_obsv_char_poly = [1, 2*zeta_obs*wn_obs, wn_obs**2]
        des_obsv_poles = np.roots(des_obsv_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(self.A.T, self.C.T)) != 2:
            print("The system is not observerable")
        else:
            self.L = cnt.acker(self.A.T, self.C.T, des_obsv_poles).T
        print('K: ', self.K)
        print('ki ', self.ki)
        print('L^T: ', self.L.T)
        #--------------------------------------------------
        # variables to implement integrator
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.x_hat = np.array([
            [0.0],  # theta_hat_0
            [0.0],  # thetadot_hat_0
        ])
        self.tau_d1 = 0.0  # control torque, delayed 1 sample

    def update(self, theta_r, y):
        # update the observer and extract theta_hat
        x_hat = self.update_observer(y)
        theta_hat = x_hat[0][0]
        # integrate error
        error = theta_r - theta_hat
        self.integrator = self.integrator \
                          + (P.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error
        # feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)
        # Compute the state feedback controller
        tau_tilde = -self.K @ x_hat - self.ki * self.integrator
        # compute total torque
        tau = saturate(tau_fl + tau_tilde[0], P.tau_max)
        self.tau_d1 = tau
        return tau, x_hat

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y_m)
        F2 = self.observer_f(self.x_hat + P.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.x_hat + P.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.x_hat + P.Ts * F3, y_m)
        self.x_hat += P.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y_m):
        # compute feedback linearizing torque tau_fl
        theta_hat = x_hat[0][0]
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta_hat)
        # xhatdot = A*(xhat-xe) + B*(u-ue) + L(y-C*xhat)
        xhat_dot = self.A @ x_hat\
                   + self.B * (self.tau_d1 - tau_fl)\
                   + self.L * (y_m - self.C @ x_hat)
        return xhat_dot


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

