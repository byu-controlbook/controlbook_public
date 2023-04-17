import numpy as np
import matplotlib.pyplot as plt
import param as P
from dynamics import dynamics
from ctrlLoop import ctrlLoop
from signalGenerator import signalGenerator
from dataPlotter import dataPlotter

# instantiate arm, controller, and reference classes
plant = dynamics()
ctrl = ctrlLoop()
reference = signalGenerator(amplitude=1.0, frequency=0.02)
disturbance = signalGenerator(amplitude=1.0)
noise = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()

t = P.t_start
y = plant.h()
while t < P.t_end:
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        r = reference.square(t)
        d = disturbance.step(t)
        n = noise.random(t)
        u = ctrl.update(r, y + n)
        y = plant.update(u + d)
        t = t + P.Ts
    # update data plots
    dataPlot.update(t, r, plant.state, u)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
