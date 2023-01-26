import numpy as np
from scipy.optimize import minimize, Bounds
import armParam as P
from signalGenerator import signalGenerator
from armDynamics import armDynamics
from ctrlPIDOpt import ctrlPID
from armSim import armSim


def optimizeGains(Q, R, controller, arm, reference, disturbance):
    gains0 = np.array([controller.kp, controller.kd, controller.ki])
    mybounds=Bounds(lb=[0,0,0], ub=[100, 100, 100])
    res = minimize(quadraticCost, 
                    x0=gains0, 
                    args=(Q, R, controller, arm, reference, disturbance), 
                    bounds=mybounds,
                    method='SLSQP')
    controller.reset(kp=res.x[0], kd=res.x[1], ki=res.x[2])
    return controller


def quadraticCost(x, Q, R, controller, arm, reference, disturbance):
    print(x)
    controller.reset(kp=x[0], kd=x[1], ki=x[2])
    arm.reset()
    t = P.t_start
    y = arm.h()
    cost = 0.
    t_end = 1/reference.frequency
    while t < t_end:  
        r = reference.square(t)
        d = disturbance.step(t)
        u = controller.update(r, y)
        y = arm.update(u+d)
        cost += Q[0][0] * (arm.state[0][0]-r)**2 \
              + Q[1][1] * (arm.state[1][0])**2 \
              + R * u**2
        t = t + P.Ts
    print(cost)
    return cost


# run optimization algorithm
arm = armDynamics()
controller = ctrlPID()
reference = signalGenerator(amplitude=30*np.pi/180.0, frequency=0.05)
disturbance = signalGenerator(amplitude=0.1)
Q = np.diag([100., 1.])
R = 0.1
controller = optimizeGains(Q, R, controller, arm, reference, disturbance)
print('kp_opt: ', controller.kp)
print('ki_opt: ', controller.ki)
print('kd_opt: ', controller.kd) 
# simulate the system using the optimized gains
armSim(controller)

