import matplotlib.pyplot as plt
import numpy as np
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from dataPlotter import dataPlotter

# instantiate reference input classes
z_fakeValueGenerator = signalGenerator(amplitude=0.5, frequency=0.1)
theta_fakeValueGenerator = signalGenerator(amplitude=.25*np.pi, frequency=.5)
f_fakeValueGenerator = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    z = z_fakeValueGenerator.sin(t)
    theta = theta_fakeValueGenerator.square(t)
    f = f_fakeValueGenerator.sawtooth(t)
    # update animation
    state = np.array([[z], [theta], [0.0], [0.0]])
    animation.update(state)
    dataPlot.update(t, state, f)
    # advance time by t_plot    
    t += P.t_plot  
    plt.pause(0.05)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
