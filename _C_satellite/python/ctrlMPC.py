import numpy as np
import affine_mpc_py as ampc
import satelliteParam as P


class MPC:
    def __init__(self):
        #  tuning parameters
        Q_diag = np.array([0.1, 1.2, 0.9, 4.5]) # smoother and less effort
        Q_diag = np.array([0.1, 6.0, 1.0, 13.0]) # in between
        Q_diag = np.array([0.0, 1.2, 0.0, 1.0]) # fast (if you don't care about theta)
        T = 200
        p = 5

        # MPC setup
        n,m = 4,1
        self.mpc = ampc.ImplicitMPC(n, m, T, p)
        A,B,w = self._get_model()
        self.mpc.setModelContinuous2Discrete(A, B, w, P.Ts)
        self.mpc.setStateWeights(Q_diag)
        u_lim = np.array([P.tau_max])
        self.mpc.setInputLimits(-u_lim, u_lim)
        self.mpc.initializeSolver()

    def update(self, x_r, x):
        self.mpc.setReferenceState(x_r)
        self.mpc.solve(x)
        u = self.mpc.getNextInput()
        return u[0]

    def _get_model(self):
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
        x_eq = np.zeros(4)
        u_eq = np.zeros(1)
        w = - (A @ x_eq + B @ u_eq)
        return A, B, w
