import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory
import armParam as P
from hw2.signalGenerator import signalGenerator
from hw2.armAnimation import armAnimation
from hw2.dataPlotter import dataPlotter
from armDynamics import armDynamics

# instantiate arm, controller, and reference classes
arm = armDynamics()
reference = signalGenerator(amplitude=0.01, frequency=0.02)
torque = signalGenerator(amplitude=0.2, frequency=0.05)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = armAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        # Get referenced inputs from signal generators
        r = reference.square(t)
        u = torque.square(t)
        y = arm.update(u)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(arm.state)
    dataPlot.update(t, r, arm.state, u)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
