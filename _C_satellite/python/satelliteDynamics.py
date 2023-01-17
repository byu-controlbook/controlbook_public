import numpy as np 
import satelliteParam as P


class satelliteDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.theta0],  # initial base angle
            [P.phi0],  # initial panel angle
            [P.thetadot0],  # initial angular velocity of base
            [P.phidot0],  # initial angular velocity of panel
        ])
        # simulation time step
        self.Ts = P.Ts
        # inertia of base
        self.Js = P.Js * (1.+alpha*(2.*np.random.rand()-1.))
        # inertia of panel
        self.Jp = P.Jp * (1.+alpha*(2.*np.random.rand()-1.))
        # spring coefficient
        self.k = P.k * (1.+alpha*(2.*np.random.rand()-1.))
        # Damping coefficient, Ns
        self.b = P.b * (1.+alpha*(2.*np.random.rand()-1.))    
        self.torque_limit = P.tau_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input torque
        u = saturate(u, self.torque_limit)
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, u):
        # Return xdot = f(x,u)
        theta = state[0][0]
        phi = state[1][0]
        thetadot = state[2][0]
        phidot = state[3][0]
        tau = u
        # The equations of motion.
        M = np.array([[self.Js, 0],
                      [0, self.Jp]])
        C = np.array([[tau - self.b*(thetadot-phidot)-self.k*(theta-phi)],
                      [-self.b*(phidot-thetadot)-self.k*(phi-theta)]])
        tmp = np.linalg.inv(M) @ C
        thetaddot = tmp[0][0]
        phiddot = tmp[1][0]
        # build xdot and return
        xdot = np.array([[thetadot], [phidot], [thetaddot], [phiddot]])
        return xdot

    def h(self):
        # return y = h(x)
        theta = self.state[0][0]
        phi = self.state[1][0]
        y = np.array([[theta], [phi]])
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
        u = limit*np.sign(u)
    return u
