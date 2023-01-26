import numpy as np 
import armParam as P


class armDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.theta0],      # initial angle
            [P.thetadot0]    # initial angular rate
        ])  
        # Mass of the arm, kg
        self.m = P.m * (1.+alpha*(2.*np.random.rand()-1.))
        # Length of the arm, m
        self.ell = P.ell * (1.+alpha*(2.*np.random.rand()-1.))
        # Damping coefficient, Ns
        self.b = P.b * (1.+alpha*(2.*np.random.rand()-1.))  
        # the gravity constant is well known, so we don't change it.
        self.g = P.g
        # sample rate at which the dynamics are propagated
        self.Ts = P.Ts  
        self.torque_limit = P.tau_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input torque
        u = saturate(u, self.torque_limit)
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y
        
    def reset(self):
        # reset the state to the initial conditions
        self.state = np.array([[P.theta0], [P.thetadot0]])

    def f(self, state, tau):
        # Return xdot = f(x,u), the system state update equations
        # re-label states for readability
        theta = state[0][0]
        thetadot = state[1][0]
        thetaddot = (3.0 / self.m / self.ell**2) * \
                    (tau - self.b*thetadot \
                     - self.m * self.g * self.ell / 2.0*np.cos(theta))
        xdot = np.array([[thetadot], [thetaddot]])
        return xdot

    def h(self):
        # return the output equations
        # could also use input u if needed
        theta = self.state[0][0]
        y = np.array([[theta]])
        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    
def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u
