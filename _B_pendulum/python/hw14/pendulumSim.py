import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import pendulumParam as P
from hw3.pendulumDynamics import pendulumDynamics
from pendulumController import pendulumController
from hw2.signalGenerator import signalGenerator
from hw2.pendulumAnimation import pendulumAnimation
from hw2.dataPlotter import dataPlotter
from hw13.dataPlotterObserver import dataPlotterObserver

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics(alpha = 0.2)
controller = pendulumController()
reference = signalGenerator(amplitude=0.5, frequency=0.05)
disturbance = signalGenerator(amplitude=0.5)
noise_z = signalGenerator(amplitude=0.01)
noise_th = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
y = pendulum.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.square(t)
        d = disturbance.step(t)  # input disturbance
        n = np.array([[noise_z.random(t)], [noise_th.random(t)]])
        u, xhat, dhat = controller.update(r, y + n)
        y = pendulum.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(pendulum.state)
    dataPlot.update(t, r, pendulum.state, u)
    dataPlotObserver.update(t, pendulum.state, xhat, d, dhat)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
