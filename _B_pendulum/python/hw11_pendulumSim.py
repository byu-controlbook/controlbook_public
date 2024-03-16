
import matplotlib.pyplot as plt
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from dataPlotter import dataPlotter
from pendulumDynamics import pendulumDynamics
from ctrlStateFeedback import ctrlStateFeedback

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics()
controller = ctrlStateFeedback()
reference = signalGenerator(amplitude=0.5, frequency=0.04)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
y = pendulum.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot: 
        r = reference.square(t)  # reference input
        x = pendulum.state
        u = controller.update(r, x)  # update controller
        y = pendulum.update(u)  # propagate system
        t += P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(pendulum.state)
    dataPlot.update(t, pendulum.state, u, r)
    plt.pause(0.0001)  

# Keeps the program from closing until user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
