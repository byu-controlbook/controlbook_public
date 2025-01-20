import numpy as np
import affine_mpc_py as ampc
import pendulumParam as P


class MPC:
    def __init__(self):
        #  tuning parameters
        Q_diag = np.array([1.0, 1.0, 0.1, 0.1])
        T = 100
        p = 5

        # MPC setup
        n,m = 4,1
        self.mpc = ampc.ImplicitMPC(n, m, T, p)
        A,B,w = self._get_model()
        self.mpc.setModelContinuous2Discrete(A, B, w, P.Ts)
        self.mpc.setStateWeights(Q_diag)
        u_lim = np.array([P.F_max])
        self.mpc.setInputLimits(-u_lim, u_lim)
        self.mpc.initializeSolver()

    def update(self, x_r, x):
        self.mpc.setReferenceState(x_r)
        self.mpc.solve(x)
        u = self.mpc.getNextInput()
        return u[0]

    def _get_model(self):
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
        x_eq = np.zeros(4)
        u_eq = np.zeros(1)
        w = - (A @ x_eq + B @ u_eq)
        return A, B, w
