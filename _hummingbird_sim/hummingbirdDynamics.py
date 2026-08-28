import numpy as np
import hummingbirdParam as P
# import eom_generated as eom

# Use the following if you saved your generated EOM functions with dill
# import dill
# dill.settings['recurse'] = True


class HummingbirdDynamics:
    def __init__(self, alpha=0.0):
        """
        Initialize a HummingbirdDynamics object.

        Args:
            alpha: parameter to vary the physical parameters of the model.
                For example, alpha=0.1 means that each parameter is varied
                by +/-10% of the nominal value.
        """
        # Initial state conditions
        self.state = np.array([
            [P.phi0],  # roll angle
            [P.theta0],  # pitch angle
            [P.psi0],  # yaw angle
            [P.phidot0],  # roll rate
            [P.thetadot0],  # pitch rate
            [P.psidot0],  # yaw rate
        ])

        # Vary the actual physical parameters
        # NOTE: if you are going to hand-code the M, C, dP_dq, and tau functions,
        # you may want to save the parameters to self instead of to a dictionary.
        self.param_vals = {
            "g": P.g, # gravity does not change
            "m_1": P.m1 * (1.+alpha*(2.*np.random.rand()-1.)),
            "m_2": P.m2 * (1.+alpha*(2.*np.random.rand()-1.)),
            "m_3": P.m3 * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_1x": P.J1x * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_1y": P.J1y * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_1z": P.J1z * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_2x": P.J2x * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_2y": P.J2y * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_2z": P.J2z * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_3x": P.J3x * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_3y": P.J3y * (1.+alpha*(2.*np.random.rand()-1.)),
            "J_3z": P.J3z * (1.+alpha*(2.*np.random.rand()-1.)),
            "ell_1": P.ell1 * (1.+alpha*(2.*np.random.rand()-1.)),
            "ell_2": P.ell2 * (1.+alpha*(2.*np.random.rand()-1.)),
            "ell_3x": P.ell3x * (1.+alpha*(2.*np.random.rand()-1.)),
            "ell_3y": P.ell3y * (1.+alpha*(2.*np.random.rand()-1.)),
            "ell_3z": P.ell3z * (1.+alpha*(2.*np.random.rand()-1.)),
            "ell_T": P.ellT * (1.+alpha*(2.*np.random.rand()-1.)),
            "d": P.d * (1.+alpha*(2.*np.random.rand()-1.)),
        }
        self.km = P.km * (1.+alpha*(2.*np.random.rand()-1.))

        # TODO: add beta to param file or define it here
        # beta = 0.001
        self.B = #TODO define the friction-based matrix of coefficients

    def update(self, u):
        """
        This is the external method that takes the input u at time
        t and returns the output y at time t.

        Args:
            u (2x1 numpy array): input vector [f_l, f_r]
        Returns:
            y (3x1 numpy array): output vector [phi, theta, psi]
        """
        u = saturate(u, P.torque_max)  # saturate the input force
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    ############################################################################
    # The following 4 methods are where we use the generated functions from
    # SymPy to calculate the terms in the equations of motion: the mass matrix
    # M, the coriolis vector C, the gravity terms dP_dq, and the generalized
    # force vector tau. Each of these methods are just wrappers to call the
    # generated functions with the appropriate parameters.
    # See the end of h3_generate_E_L.py for an example of how to call the
    # generated functions.

    # NOTE: IF YOU DID NOT GENERATE THESE FUNCTIONS, YOU WOULD NEED TO
    # IMPLEMENT THE CALCULATION OF THESE TERMS BY HAND.

    # If you saved the generated functions with dill, you can load them like this:
    #   M_func = dill.load(open("hb_M_func.pkl", "rb"))
    #   C_func = dill.load(open("hb_C_func.pkl", "rb"))
    #   dP_dq_func = dill.load(open("hb_dP_dq_func.pkl", "rb"))
    #   tau_func = dill.load(open("hb_tau_func.pkl", "rb"))

    def calculate_M(self, state):
        # TODO: Fill in this function
        # M = eom.calculate_M(state, **self.param_vals)
        return M

    def calculate_C(self, state):
        # TODO: Fill in this function
        # C = eom.calculate_C(state, **self.param_vals)
        return C

    def calculate_dP_dq(self, state):
        # TODO: Fill in this function
        # dP_dq = eom.calculate_dP_dq(state, **self.param_vals)
        return dP_dq

    def calculate_tau(self, state, u):
        # TODO: Fill in this function
        # tau = eom.calculate_tau(state, u, **self.param_vals)
        return tau
    ############################################################################

    def f(self, state, u):
        """
        return xdot = f(x,u)

        Args:
            state (6x1 array): state [phi, theta, psi, phidot, thetadot, psidot]
            u (2x1 array): input [f_l, f_r]
        Returns:
            xdot (6x1 array): state derivative
        """
        # TODO: call the functions to calculate M, C, dP_dq, and tau to
        # numerically evaluate them at the current point in time.
        M = self.calculate_M()
        C = self.calculate_C()
        dP_dq = self.calculate_dP_dq()
        tau = self.calculate_tau()

        # TODO: write an expression for qddot from the lab manual equations,
        # remember that it will be in terms of M, C, dP_dq, -Bqdot, and tau
        # (but all of them will be numerical values, not functions)
        qddot =

        # Pull out the first derivative (or velocity-based) terms from the state
        phidot = state[3, 0]
        thetadot = state[4, 0]
        psidot = state[5, 0]

        # Define the second derivatives from qddot
        phiddot = qddot[0, 0]
        thetaddot = qddot[1, 0]
        psiddot = qddot[2, 0]

        # Build xdot and return
        xdot = np.array([[phidot],
                         [thetadot],
                         [psidot],
                         [phiddot],
                         [thetaddot],
                         [psiddot]])
        return xdot

    def h(self):
        """
        Output function.

        Returns:
            y (3x1 array): output vector [phi, theta, psi]
        """
        # TODO Fill in this function using self.state
        phi =
        theta =
        psi =
        y = np.array([[phi], [theta], [psi]])
        return y

    def rk4_step(self, u):
        """
        Integrate ODE using Runge-Kutta RK4 algorithm.

        Args:
            u (2x1 array): input vector [f_l, f_r]
        """
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + P.Ts / 2 * F1, u)
        F3 = self.f(self.state + P.Ts / 2 * F2, u)
        F4 = self.f(self.state + P.Ts * F3, u)
        self.state = self.state + P.Ts / 6 * (F1 + 2*F2 + 2*F3 + F4)


def saturate(u, limit):
    for i in range(0, u.shape[0]):
        if abs(u[i,0]) > limit:
            u[i,0] = limit * np.sign(u[i,0])
    return u
