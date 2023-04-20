import matplotlib.pyplot as plt
import numpy as np
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from dataPlotter import dataPlotter
from pendulumDynamics import pendulumDynamics
from ctrlLoopshape import ctrlLoopshape

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics(alpha = 0.1)
controller = ctrlLoopshape()
reference = signalGenerator(amplitude=0.5, frequency=0.04)
disturbance = signalGenerator(amplitude=0.5)
noise_z = signalGenerator(amplitude=0.001)
noise_th = signalGenerator(amplitude=0.001)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
y = pendulum.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        r = reference.square(t)
        d = 1.0  #input disturbance
        n = np.array([[noise_z.random(t)], [noise_th.random(t)]])
        u = controller.update(r, y + n) 
        y = pendulum.update(u + d)  
        t = t + P.Ts  
    # update animation and data plots
    animation.update(pendulum.state)
    dataPlot.update(t, r, pendulum.state, u)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
