import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory
import armParam as P
from signalGenerator import signalGenerator
from armAnimation import armAnimation
from plotData import plotData
from armDynamics import armDynamics

# instantiate arm, controller, and reference classes
arm = armDynamics()
reference = signalGenerator(amplitude=0.01, frequency=0.02)
torque = signalGenerator(amplitude=0.2, frequency=0.05)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = armAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        tau=torque.square(t)
        arm.propagateDynamics(tau)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.drawArm(arm.states())
    dataPlot.updatePlots(t, ref_input, arm.states(), tau)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
