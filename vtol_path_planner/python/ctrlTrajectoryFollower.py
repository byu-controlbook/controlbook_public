import numpy as np
import VTOLParam as P
import scipy 
#from scipy import signal
#import control as cnt


class ctrlTrajectoryFollower:
    def __init__(self):
        # tuning parameters
        wn_z    = 0.5 #0.9905
        zeta_z  = 0.707
        wn_h    = 1.0 #1.5
        zeta_h  = 0.707
        wn_th   = 9. #13.3803
        zeta_th = 0.707
        integrator_h = 0.2
        integrator_z = 0.2
        # State Space Equations
        self.Fe = (P.mc + 2.0 * P.mr) * P.g  # equilibrium force 
        A = np.array([[0., 0., 0., 1., 0., 0.],
                      [0., 0., 0., 0., 1., 0.],
                      [0., 0., 0., 0., 0., 1.],
                      [0., 0., -self.Fe / (P.mc + 2.0 * P.mr), -(P.mu / (P.mc + 2.0 * P.mr)), 0., 0. ],
                      [0., 0., 0., 0., 0., 0.],
                      [0., 0., 0., 0., 0., 0.]])
        B = np.array([[0., 0.],
                      [0., 0.],
                      [0., 0.],
                      [0., 0.],
                      [1.0 / (P.mc + 2.0 * P.mr), 0.],
                      [0., 1.0 / (P.Jc + 2 * P.mr * P.d ** 2)]])
        self.C = np.array([[1., 0., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0., 0.]])
        # form augmented system
        A1 = np.vstack((
                np.hstack((A, np.zeros((6,2)))),
                np.hstack((self.C, np.zeros((2,2))))))
        B1 = np.vstack((B, np.zeros((2,2))))
        # gain calculation
        des_char_poly = np.convolve(np.convolve(np.convolve(np.convolve(
            [1.0, 2.0 * zeta_z * wn_z, wn_z ** 2],
            [1.0, 2.0 * zeta_h * wn_h, wn_h ** 2]), 
            [1.0, 2.0 * zeta_th * wn_th, wn_th ** 2]),
            [1, integrator_z]),
            [1, integrator_h])
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if 0: #np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 8:
            print("The system is not controllable")
        else:
            #K1 = cnt.place(A1, B1, des_poles)
            res = scipy.signal.place_poles(A1, B1, des_poles)
            K1 = res.gain_matrix
            self.K = K1[0:2, 0:6]
            self.KI = K1[0:2, 6:8]
        self.integrator = np.array([[0.], [0]])  
        self.error_d1 = np.array([[0.], [0.]]) 
        self.Ts = P.Ts

    def update(self, path, x):
        x_r = path[0:6]
        x_tilde = x - x_r
        u_r = path[6:8]
        # integrate error
        error = self.C @ x_tilde
        self.integrator += (P.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error
        # Compute the state feedback controllers
        u = np.array([[self.Fe], [0.]]) + u_r \
            - self.K @ x_tilde - self.KI @ self.integrator
        return u


def saturate(u, limit):
    if abs(u) > limit:
        u = limit*np.sign(u)
    return u

