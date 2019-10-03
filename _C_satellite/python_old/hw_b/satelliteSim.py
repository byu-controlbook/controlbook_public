import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P
from signalGenerator import signalGenerator
from satelliteAnimation import satelliteAnimation
from plotData import plotData
from satelliteDynamics import satelliteDynamics

# instantiate satellite, controller, and reference classes
satellite = satelliteDynamics()
reference = signalGenerator(amplitude=0.5, frequency=0.1)
torque = signalGenerator(amplitude=0.1, frequency=0.1)

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
        tau = torque.sin(t)
        satellite.propagateDynamics(tau)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.drawSatellite(satellite.states())
    dataPlot.updatePlots(t, ref_input, satellite.states(), tau)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
