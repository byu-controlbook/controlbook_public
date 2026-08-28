import matplotlib.pyplot as plt
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from dataPlotter import dataPlotter
from pendulumDynamics import pendulumDynamics
from ctrlPID import ctrlPID

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics(alpha=0.2)
controller = ctrlPID()
reference = signalGenerator(amplitude=0.5, frequency=0.04)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
y = pendulum.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.square(t)  # reference input
        u = controller.update(r, y)  # update controller
        y = pendulum.update(u)  # propagate system
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(pendulum.state)
    dataPlot.update(t, pendulum.state, u, r)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
