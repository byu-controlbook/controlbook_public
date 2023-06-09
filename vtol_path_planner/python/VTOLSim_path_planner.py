import matplotlib.pyplot as plt
import numpy as np
import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from VTOLDynamics import VTOLDynamics
from dataPlotterObserver import dataPlotterObserver
from ctrlDisturbanceObserver import ctrlDisturbanceObserver
from pathPlanner import pathPlanner

# instantiate VTOL, controller, and reference classes
VTOL = VTOLDynamics()
controller = ctrlDisturbanceObserver()
planner = pathPlanner()
z_noise = signalGenerator(amplitude=0.01)
h_noise = signalGenerator(amplitude=0.01)
th_noise = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
y = VTOL.h()  # output of system at start of simulation
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        planner.update(t)
        d = np.array([[0.],[0.]])
        n = np.array([[z_noise.random(t)],
                      [h_noise.random(t)],
                      [th_noise.random(t)]])
        u, xhat_lat, xhat_lon = controller.update(planner.path, y + n)  # update controller
        y = VTOL.update(P.mixing @ (u + d))  # propagate system
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    dataPlot.update(t, VTOL.state, planner.path[0][0], planner.path[1][0], u)
    dataPlotObserver.update(t, VTOL.state, xhat_lat, xhat_lon)
    animation.update(VTOL.state, 0.0)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
