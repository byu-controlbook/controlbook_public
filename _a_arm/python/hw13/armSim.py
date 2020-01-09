import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import armParam as P
from hw3.armDynamics import armDynamics
from armController import armController
from hw2.signalGenerator import signalGenerator
from hw2.armAnimation import armAnimation
from hw2.dataPlotter import dataPlotter
from dataPlotterObserver import dataPlotterObserver

# instantiate arm, controller, and reference classes
arm = armDynamics(alpha=0.0)
controller = armController()
reference = signalGenerator(amplitude=30*np.pi/180.0, frequency=0.05)
disturbance = signalGenerator(amplitude=0.25)
noise = signalGenerator(amplitude=0.01)

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
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        r = reference.square(t)
        d = 0  #disturbance.step(t)  # input disturbance
        n = 0  #noise.random(t)  # simulate sensor noise
        u, xhat = controller.update(r, y + n)  # update controller
        y = arm.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(arm.state)
    dataPlot.update(t, r, arm.state, u)
    dataPlotObserver.update(t, arm.state, xhat, d, 0.0)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
