import numpy as np
import VTOLParam as P
from scipy import signal
import control as cnt


class ctrlDisturbanceObserver:
    def __init__(self):
        # tuning parameters
        wn_h    = 1.0
        zeta_h  = 0.707
        wn_z    = 0.9905
        zeta_z  = 0.707
        wn_th   = 13.3803
        zeta_th = 0.707
        integrator_h = 3.0
        integrator_z = 2.0
        # observer gains
        wn_h_obs    = 10.0*wn_h
        wn_z_obs    = 10.0*wn_z
        wn_th_obs   = 5.0*wn_th
        dist_obsv_pole_lon = 10.0
        dist_obsv_pole_lat = 10.0
        # State Space Equations
        self.Fe = (P.mc + 2.0 * P.mr) * P.g  # equilibrium force 
        A_lon = np.array([[0.0, 1.0],
                        [0.0, 0.0]])
        B_lon = np.array([[0.0],
                        [1.0 / (P.mc + 2.0 * P.mr)]])
        C_lon = np.array([[1.0, 0.0]])
        A_lat = np.array([[0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                        [0.0, -self.Fe / (P.mc + 2.0 * P.mr), \
                         -(P.mu / (P.mc + 2.0 * P.mr)), 0.0],
                        [0.0, 0.0, 0.0, 0.0]])
        B_lat = np.array([[0.0],
                        [0.0],
                        [0.0],
                        [1.0 / (P.Jc + 2 * P.mr * P.d ** 2)]])
        C_lat = np.array([[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0]])
        # form augmented system
        A1_lon = np.vstack((
                np.hstack((A_lon, np.zeros((2,1)))),
                np.hstack((-C_lon, np.zeros((1,1))))))
        self.B1_lon = np.vstack((B_lon, np.zeros((1,1))))
        A1_lat = np.vstack((
                np.hstack((A_lat, np.zeros((4,1)))),
                np.hstack((-C_lat[0:1], np.zeros((1,1))))))
        self.B1_lat = np.vstack((B_lat, np.zeros((1,1))))
        # gain calculation
        des_char_poly_lon = np.convolve([1.0, 2.0 * zeta_h * wn_h, wn_h ** 2],
                                        [1, integrator_h])
        des_poles_lon = np.roots(des_char_poly_lon)
        des_char_poly_lat = np.convolve(
            np.convolve([1.0, 2.0 * zeta_z * wn_z, wn_z ** 2],
                        [1.0, 2.0 * zeta_th * wn_th, wn_th ** 2]),
            [1, integrator_z])
        des_poles_lat = np.roots(des_char_poly_lat)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1_lon, self.B1_lon)) != 3:
            print("The longitudinal system is not controllable")
        else:
            K1_lon = cnt.place(A1_lon, self.B1_lon, des_poles_lon)
            self.K_lon = K1_lon[0][0:2]
            self.ki_lon = K1_lon[0][2]
        if np.linalg.matrix_rank(cnt.ctrb(A1_lat, self.B1_lat)) != 5:
            print("The lateral system is not controllable")
        else:
            K1_lat = cnt.place(A1_lat, self.B1_lat, des_poles_lat)
            self.K_lat = K1_lat[0][0:4]
            self.ki_lat = K1_lat[0][4]
        # form augmented system for observer design
        self.A2_lon = np.concatenate((
                np.concatenate((A_lon, B_lon), axis=1),
                np.zeros((1, 3))),
                axis=0)
        self.C2_lon = np.concatenate((C_lon, np.zeros((1, 1))), axis=1)
        self.A2_lat = np.concatenate((
                np.concatenate((A_lat, B_lat), axis=1),
                np.zeros((1, 5))),
                axis=0)
        self.C2_lat = np.concatenate((C_lat, np.zeros((2, 1))), axis=1)
        # compute observer poles
        des_obs_char_poly_lon = np.convolve([1.0, 2.0*zeta_h*wn_h_obs, wn_h_obs**2],
                                            [1, dist_obsv_pole_lon])
        des_obs_poles_lon = np.roots(des_obs_char_poly_lon)
        des_obs_char_poly_lat = np.convolve(
                                    np.convolve([1.0, 2.0*zeta_z*wn_z_obs, wn_z_obs**2],
                                                [1.0, 2.0*zeta_th*wn_th_obs, wn_th_obs**2]),
                                    [1, dist_obsv_pole_lat])
        des_obs_poles_lat = np.roots(des_obs_char_poly_lat)
        if np.linalg.matrix_rank(cnt.ctrb(self.A2_lon.T, self.C2_lon.T)) != 3:
            print("The longitudinal system is not observable")
        else:
            self.L2_lon = signal.place_poles(self.A2_lon.T, self.C2_lon.T, des_obs_poles_lon).gain_matrix.T
        if np.linalg.matrix_rank(cnt.ctrb(self.A2_lat.T, self.C2_lat.T)) != 5:
            print("The lateral system is not observable")
        else:
            self.L2_lat = signal.place_poles(self.A2_lat.T, self.C2_lat.T, des_obs_poles_lat).gain_matrix.T
        print('K_lon: ', self.K_lon)
        print('ki_lon: ', self.ki_lon)
        print('L_lon^T: ', self.L2_lon.T)
        print('K_lat: ', self.K_lat)
        print('ki_lat: ', self.ki_lat)
        print('L_lat^T: ', self.L2_lat.T)
        self.xhat_lon = np.array([[0.0], [0.0], [0.0]])
        self.xhat_lat = np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
        self.integrator_z = 0.0      # integrator on position z
        self.error_z_d1 = 0.0        # error signal delayed by 1 sample
        self.integrator_h = 0.0      # integrator on altitude h
        self.error_h_d1 = 0.0        # error signal delayed by 1 sample
        self.F_d1 = 0.0  # Force signal delayed by 1 sample
        self.tau_d1 = 0.0  # torque signal delayed by 1 sample
        self.F_limit = P.max_thrust * 2.0
        self.tau_limit = P.max_thrust * P.d * 2.0
        self.Ts = P.Ts

    def update(self, path, y):
        z_r = path[0][0]
        h_r = path[1][0]
        y_lat = np.array([[y[0][0]],[y[2][0]]])
        y_lon = y[1][0]
        # update the observers
        xhat_lat, dhat_lat = self.update_lat_observer(y_lat)
        xhat_lon, dhat_lon = self.update_lon_observer(y_lon)
        z_hat = xhat_lat[0][0]
        h_hat = xhat_lon[0][0]
        theta_hat = xhat_lat[1][0]
        # integrate error
        error_z = z_r - z_hat
        self.integrator_z += (P.Ts/2.0)*(error_z + self.error_z_d1)
        self.error_z_d1 = error_z
        error_h = h_r - h_hat
        self.integrator_h += (P.Ts/2.0)*(error_h + self.error_h_d1)
        self.error_h_d1 = error_h
        # Compute the state feedback controllers
        F_tilde = -self.K_lon @ xhat_lon - self.ki_lon * self.integrator_h
        F = self.Fe / np.cos(theta_hat) + F_tilde[0] - dhat_lon
        F_sat = saturate(F, self.F_limit)
        self.integratorAntiWindup(F_sat, F, self.ki_lon, self.integrator_h)
        tau = -self.K_lat @ xhat_lat - self.ki_lat*self.integrator_z - dhat_lat
        tau_sat = saturate(tau[0], self.tau_limit)
        self.integratorAntiWindup(tau_sat, tau, self.ki_lat, self.integrator_z)
        u = np.array([[F_sat], [tau_sat]])
        self.F_d1 = F_sat
        self.tau_d1 = tau_sat
        return u, xhat_lat, xhat_lon

    def integratorAntiWindup(self, u_sat, u_unsat, ki, integrator):
        if ki != 0.0:
            integrator = integrator + P.Ts/ki*(u_sat-u_unsat)

    def update_lat_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lat(self.xhat_lat, y_m)
        F2 = self.observer_f_lat(self.xhat_lat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lat(self.xhat_lat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lat(self.xhat_lat + self.Ts * F3, y_m)
        self.xhat_lat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        xhat = np.array([[self.xhat_lat.item(0)],
                         [self.xhat_lat.item(1)],
                         [self.xhat_lat.item(2)],
                         [self.xhat_lat.item(3)],
                        ])
        dhat = self.xhat_lat.item(4)
        return xhat, dhat

    def observer_f_lat(self, x_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = self.A2_lat @ x_hat \
                   + self.B1_lat * self.tau_d1 \
                   + self.L2_lat @ (y_m - self.C2_lat @ x_hat)
        return xhat_dot

    def update_lon_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lon(self.xhat_lon, y_m)
        F2 = self.observer_f_lon(self.xhat_lon + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lon(self.xhat_lon + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lon(self.xhat_lon + self.Ts * F3, y_m)
        self.xhat_lon += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        xhat = np.array([[self.xhat_lon.item(0)],
                         [self.xhat_lon.item(1)],
                        ])
        dhat = self.xhat_lon.item(2)
        return xhat, dhat

    def observer_f_lon(self, x_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = self.A2_lon @ x_hat \
                   + self.B1_lon * (self.F_d1 - self.Fe) \
                   + self.L2_lon @ (y_m - self.C2_lon @ x_hat)
        return xhat_dot


def saturate(u, limit):
    if abs(u) > limit:
        u = limit*np.sign(u)
    return u

