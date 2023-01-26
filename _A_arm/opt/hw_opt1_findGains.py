import numpy as np
import armParam as P
from scipy.optimize import minimize
from signalGenerator import signalGenerator
from armDynamics import armDynamics
from ctrlPIDOpt import ctrlPID

# instantiate arm, controller, and reference classes
arm = armDynamics(alpha=0.)
controller = ctrlPID()
reference = signalGenerator(amplitude=30*np.pi/180.0)

def quadratic_cost(x, Q, R);
    controller.reset(kp=x[0], kd=x[1], ki=x[2])
    t = 0.
    y = arm.h()
    cost = 0.
    while t < P.t_end:  
        r = reference.step(t)
        u = controller.update(r, y)
        y = arm.update(u)
        cost += Q[0][0] * (arm.state[0][0]-r)**2 \
              + Q[1][1] * (arm.state[1][0])**2 \
              + R * u**2
        t = t + P.Ts
    return cost

if __name__ == "__main__":
    x0 = np.array([controller.kp, controller.kd, controller.ki])
    res = minimize('quadratic_cost', x0, args=(controller, arm, Q, R), method=SLSQP)
    controller.reset(kp=res.x[0], kd=res.x[1], ki=res.x[2])

