import matplotlib.pyplot as plt
import numpy as np
import armParam as P
from signalGenerator import signalGenerator
from armAnimation import armAnimation
from dataPlotter import dataPlotter
from armDynamics import armDynamics
from ctrlObserver import ctrlObserver
from dataPlotterObserver import dataPlotterObserver

# instantiate arm, controller, and reference classes
arm = armDynamics(alpha=0.0)
controller = ctrlObserver()
reference = signalGenerator(amplitude=30*np.pi/180.0,
                            frequency=0.05)
disturbance = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = armAnimation()

t = P.t_start  # time starts at t_start
y = arm.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot: 
        r = reference.square(t)
        d = disturbance.step(t) # start with no d,
                                # then use this for part e)

        u, xhat = controller.update(r, y)  # update controller
        y = arm.update(u + d)  # propagate system
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(arm.state)
    dataPlot.update(t, arm.state, u, r)
    dataPlotObserver.update(t, arm.state, xhat, d, 0.0)

    # the pause causes the figure to display during simulation
    plt.pause(0.0001)  

# Keeps the program from closing until user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
