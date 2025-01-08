import matplotlib.pyplot as plt
import numpy as np
import satelliteParam as P
from signalGenerator import signalGenerator
from satelliteAnimation import satelliteAnimation
from dataPlotter import dataPlotter
from satelliteDynamics import satelliteDynamics
from ctrlMPC import MPC


satellite = satelliteDynamics()
controller = MPC()
reference = signalGenerator(amplitude=np.radians(15), frequency=0.04)

dataPlot = dataPlotter()
animation = satelliteAnimation()

t = P.t_start
y = satellite.h()
while t < P.t_end:
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        phi_r = reference.square(t)
        x_r = np.array([phi_r, phi_r, 0.0, 0.0])
        x = satellite.state # could use an estimator instead of true state
        u = controller.update(x_r, x)
        y = satellite.update(u)
        t += P.Ts

    animation.update(satellite.state)
    dataPlot.update(t, satellite.state, u, phi_r)
    plt.pause(0.0001) # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
