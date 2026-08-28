import matplotlib.pyplot as plt
import numpy as np
import armParam as P
from signalGenerator import signalGenerator
from armAnimation import armAnimation
from dataPlotter import dataPlotter

# instantiate reference input classes, these are not actual values,
# just values to allow us to plot
theta_plot = signalGenerator(amplitude=2.0*np.pi, frequency=0.1)
tau_plot = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = armAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    theta = theta_plot.sin(t)
    tau = tau_plot.sawtooth(t)

    # update animation
    theta_dot = 0.0

    # state is made of theta, and theta_dot
    state = np.array([[theta], [theta_dot]])  

    # update animation and plot 
    animation.update(state)
    dataPlot.update(t, state, tau)

    # advance time by t_plot
    t += P.t_plot
    plt.pause(0.02)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
