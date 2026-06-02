import numpy as np
import control as cnt
import satelliteParam as P


class ctrlStateFeedbackIntegrator:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        tr_th = 2.0
        M = 3.0  # Time scale separation between loops
        tr_phi = M * tr_th  # rise time for outer loop
        zeta_th = 0.9  # damping ratio for theta
        zeta_phi = 0.9  # damping ratio for phi
        integrator_pole = -2.0

        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [-P.k/P.Js, P.k/P.Js, -P.b/P.Js, P.b/P.Js],
                      [P.k/P.Jp, -P.k/P.Jp, P.b/P.Jp, -P.b/P.Jp]])
        B = np.array([[0.0],
                      [0.0],
                      [1.0 / P.Js],
                      [0.0]])
        C = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        
        # form augmented system
        Cr = np.array([[0.0, 1.0, 0.0, 0.0]])
        A1 = np.vstack((np.hstack((A, np.zeros((np.size(A,1),1)))), 
                        np.hstack((-Cr, np.array([[0.0]]))) ))
        B1 = np.vstack( (B, 0.0) )

        # gain calculation
        wn_th = 0.5*np.pi/(tr_th*np.sqrt(1-zeta_th**2)) 
        wn_phi = 0.5*np.pi/(tr_phi*np.sqrt(1-zeta_phi**2)) 
        des_char_poly = np.convolve(
                np.convolve([1, 2 * zeta_phi * wn_phi, wn_phi**2],
                            [1, 2 * zeta_th * wn_th, wn_th**2]),
                [1, -integrator_pole])
        des_poles = np.roots(des_char_poly)

        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            print("The system is not controllable")
        else:
            K1 = cnt.place(A1, B1, des_poles)
            self.K = K1[:, 0:4]
            self.ki = K1[0, 4]
        print('K: ', self.K)
        print('ki: ', self.ki)

        # variables to implement integrator
        self.integrator_phi = 0.0  # integrator
        self.error_phi_d1 = 0.0  # error signal delayed by 1 sample

    def update(self, phi_r, x):
        phi = x[0, 0]

        # integrate error
        error_phi = phi_r - phi

        # integrate error
        self.integrator_phi = self.integrator_phi \
            + (P.Ts / 2.0) * (error_phi + self.error_phi_d1)
        self.error_phi_d1 = error_phi

        # Compute the state feedback controller
        tau_unsat = -self.K @ x - self.ki * self.integrator_phi
        tau = saturate(tau_unsat[0, 0], P.tau_max)
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

