import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from plotData import plotData
from pendulumDynamics import pendulumDynamics

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics()
reference = signalGenerator(amplitude=0.5, frequency=0.02)
force = signalGenerator(amplitude=1, frequency=1)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        f = force.sin(t)
        pendulum.propagateDynamics(f)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.drawPendulum(pendulum.states())
    dataPlot.updatePlots(t, ref_input, pendulum.states(), f)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
