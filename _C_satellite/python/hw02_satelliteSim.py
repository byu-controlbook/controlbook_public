import matplotlib.pyplot as plt
import numpy as np
import satelliteParam as P
from signalGenerator import signalGenerator
from satelliteAnimation import satelliteAnimation
from dataPlotter import dataPlotter

# instantiate reference input classes
theta_fakeValueGenerator = signalGenerator(amplitude=2.0*np.pi, frequency=0.1)
phi_fakeValueGenerator = signalGenerator(amplitude=0.5, frequency=0.1)
tau_fakeValueGenerator = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = satelliteAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    theta = theta_fakeValueGenerator.sin(t)
    phi = phi_fakeValueGenerator.sin(t)
    tau = tau_fakeValueGenerator.sawtooth(t)

    # update animation
    state = np.array([[theta], [phi], [0.0], [0.0]])
    animation.update(state)
    dataPlot.update(t, state, tau)
    
    # advance time by t_plot
    t += P.t_plot
    plt.pause(0.02)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
