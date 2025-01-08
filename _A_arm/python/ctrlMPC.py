import numpy as np
import affine_mpc_py as ampc
import armParam as P


class MPC:
    def __init__(self):
        #  tuning parameters
        Q_diag = np.array([1.0, 0.1])
        T = 100
        p = 5

        # MPC setup
        n,m = 2,1
        self.mpc = ampc.ImplicitMPC(n, m, T, p)
        A,B,w = self._linearize_model(np.zeros(n))
        self.mpc.setModelContinuous2Discrete(A, B, w, P.Ts)
        self.mpc.setStateWeights(Q_diag)
        u_lim = np.array([P.tau_max])
        self.mpc.setInputLimits(-u_lim, u_lim)
        self.mpc.initializeSolver()

    def _linearize_model(self, x_eq):
        den = P.m * P.ell**2
        A = np.array([[0, 1],
                      [0, -3*P.b / den]])
        B = np.array([[0],
                      [3 / den]])
        x_eq = np.array([x_eq.item(0), 0])
        u_eq = self._get_equilibrium_input(x_eq)
        w = - (A @ x_eq + B @ u_eq)
        return A, B, w

    def _get_equilibrium_input(self, x):
        tau_eq = 0.5 * P.m * P.g * P.ell * np.cos(x[0])
        return np.array([tau_eq])

    def update(self, x_r, x):
        self.mpc.setReferenceState(x_r)
        A,B,w = self._linearize_model(x)
        self.mpc.setModelContinuous2Discrete(A, B, w, P.Ts)
        self.mpc.solve(x)
        u = self.mpc.getNextInput()
        return u[0]
