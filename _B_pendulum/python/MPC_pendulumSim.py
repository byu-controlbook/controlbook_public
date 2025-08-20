import matplotlib.pyplot as plt
import numpy as np
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from dataPlotter import dataPlotter
from pendulumDynamics import pendulumDynamics
from ctrlMPC import MPC


pendulum = pendulumDynamics()
controller = MPC()
reference = signalGenerator(amplitude=0.5, frequency=0.04)

dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start
y = pendulum.h()
while t < P.t_end:
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        z_r = reference.square(t)
        x_r = np.array([z_r, 0.0, 0.0, 0.0])
        x = pendulum.state # could use an estimator instead of true state
        u = controller.update(x_r, x)
        y = pendulum.update(u)
        t += P.Ts

    animation.update(pendulum.state)
    dataPlot.update(t, pendulum.state, u, z_r)
    plt.pause(0.0001) # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until user presses a button
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
