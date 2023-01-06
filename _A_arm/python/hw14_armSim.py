import matplotlib.pyplot as plt
import numpy as np
import armParam as P
from signalGenerator import signalGenerator
from armAnimation import armAnimation
from dataPlotter import dataPlotter
from armDynamics import armDynamics
from dataPlotterObserver import dataPlotterObserver
from ctrlDisturbanceObserver import ctrlDisturbanceObserver

# instantiate arm, controller, and reference classes
arm = armDynamics(alpha=0.2)
controller = ctrlDisturbanceObserver()
reference = signalGenerator(amplitude=30*np.pi/180.0,
                            frequency=0.05)
disturbance = signalGenerator(amplitude=0.5)
noise = signalGenerator(amplitude=0.001)

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
        d = disturbance.step(t)  # input disturbance
        n = noise.random(t)  # simulate sensor noise

        # update controller
        u, xhat, dhat = controller.update(r, y + n)  
        y = arm.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(arm.state)
    dataPlot.update(t, r, arm.state, u)
    dataPlotObserver.update(t, arm.state, xhat, d, dhat)
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
