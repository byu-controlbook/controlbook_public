import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import satelliteParam as P
from satelliteDynamics import satelliteDynamics
from satelliteController import satelliteController
from signalGenerator import signalGenerator
from satelliteAnimation import satelliteAnimation
from plotData import plotData

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
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        tau = ctrl.u(ref_input, satellite.outputs())  # Calculate the control value
        sys_input = [tau[0]+disturbance]  # input to plant is control input + disturbance (formatted as a list)
        satellite.propagateDynamics(sys_input)  # Propagate the dynamics with disturbance input
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.drawSatellite(satellite.states())
    dataPlot.updatePlots(t, ref_input, satellite.states(), tau)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
