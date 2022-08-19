######## this file is incomplete, but would be used to give more intuition behind
######## what frequency response is by just simulating different sine inputs on arm.
###### This is the functionality to change the freq. that is missing - https://matplotlib.org/stable/gallery/widgets/slider_demo.html

import sys
sys.path.append('..')  # add parent directory
sys.path.append('../hw10')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import armParam as P
from hw3.armDynamics import armDynamics
from hw10.armController import armController
from hw2.signalGenerator import signalGenerator
from hw2.armAnimation import armAnimation
from hw2.dataPlotter import dataPlotter

# instantiate arm, controller, and reference classes
arm = armDynamics()
controller = armController()
reference = signalGenerator(amplitude=30*np.pi/180.0,
                            frequency=10.0)
disturbance = signalGenerator(amplitude=0.0)
noise = signalGenerator(amplitude=0.1, frequency=60)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = armAnimation()

t = P.t_start  # time starts at t_start
y = arm.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop

    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:
        r = reference.sin(t)
        d = 0.0 #disturbance.step(t)  # input disturbance
        n = 0.0 # noise.sin(t)  #noise.random(t)  # simulate sensor noise
        u = controller.update(r, y + n)  # update controller
        y = arm.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(arm.state)
    dataPlot.update(t, r, arm.state, u)

    # the pause causes the figure to display during simulation
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
