import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import satelliteParam as P
from hw8.satelliteDynamics import satelliteDynamics
from satelliteController import satelliteController
from hw_a.signalGenerator import signalGenerator
from hw_a.satelliteAnimation import satelliteAnimation
from hw_a.plotData import plotData

# instantiate satellite, controller, and reference classes
satellite = satelliteDynamics()
ctrl = satelliteController()
reference = signalGenerator(amplitude=15.0*np.pi/180.0, frequency=0.02)

# set disturbance input
disturbance = 0.0

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = satelliteAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        u = ctrl.update(ref_input, satellite.state)  # Calculate the control value
        satellite.update(u)  # Propagate the dynamics with disturbance input
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(satellite.state)
    dataPlot.update(t, ref_input, satellite.state, u)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
