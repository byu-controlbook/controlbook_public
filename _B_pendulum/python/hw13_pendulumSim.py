import matplotlib
matplotlib.use('tkAgg')
import matplotlib.pyplot as plt
import numpy as np
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from dataPlotter import dataPlotter
from pendulumDynamics import pendulumDynamics
from ctrlObserver import ctrlObserver
from dataPlotterObserver import dataPlotterObserver

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics(alpha=0.0)
controller = ctrlObserver()
reference = signalGenerator(amplitude=0.5, frequency=0.05)
disturbance = signalGenerator(amplitude=0.05)

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
        d = disturbance.step(t)  # this is 0 for part a), 
                                 # then use this for part e)
        u, xhat = controller.update(r, y)
        y = pendulum.update(u + d)  # propagate system
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(pendulum.state)
    dataPlot.update(t, pendulum.state, u, r)
    dataPlotObserver.update(t, pendulum.state, xhat, d, 0.0)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
