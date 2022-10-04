import matplotlib.pyplot as plt
import numpy as np
import pendulumParam as P
from hw2.signalGenerator import signalGenerator
from hw2.pendulumAnimation import pendulumAnimation
from hw2.dataPlotter import dataPlotter
from hw3.pendulumDynamics import pendulumDynamics
from hw8.pendulumController import pendulumController

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics()
controller = pendulumController()
reference = signalGenerator(amplitude=0.5, frequency=0.08)
disturbance = signalGenerator(amplitude=0)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
y = pendulum.h()  # output of system at start of simulation

# for part e), we can uncomment below
#pendulum.state[1,0] = 10.0*np.pi/180.0
#reference = signalGenerator(amplitude = 0.0, frequency=0.0)


while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.step(t)  # reference input
        d = disturbance.step(t)  # input disturbance
        n = 0.0  #noise.random(t)  # simulate sensor noise
        x = pendulum.state  # use state instead of output
        u = controller.update(r, x)  # update controller
        y = pendulum.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(pendulum.state)
    dataPlot.update(t, r, pendulum.state, u)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
