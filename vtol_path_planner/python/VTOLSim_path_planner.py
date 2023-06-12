import matplotlib.pyplot as plt
import numpy as np
import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from VTOLDynamics import VTOLDynamics
from ctrlTrajectoryFollower import ctrlTrajectoryFollower
from pathPlanner import pathPlanner

# instantiate VTOL, controller, and reference classes
VTOL = VTOLDynamics()
controller = ctrlTrajectoryFollower()
planner = pathPlanner()

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
y = VTOL.h()  # output of system at start of simulation
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        planner.update(t)
        u = controller.update(planner.path, VTOL.state)  # update controller
        y = VTOL.update(P.mixing @ u)  # propagate system
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    dataPlot.update(t, VTOL.state, planner.path, u)
    animation.update(VTOL.state, 0.0)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
