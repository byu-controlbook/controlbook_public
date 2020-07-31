import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P
from signalGenerator import signalGenerator
from satelliteAnimation import satelliteAnimation
from dataPlotter import dataPlotter


# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
thetaRef = signalGenerator(amplitude=2.0*np.pi, frequency=0.1)
phiRef = signalGenerator(amplitude=0.5, frequency=0.1)
tauRef = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = satelliteAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop

    # set variables
    r = reference.square(t)
    theta = thetaRef.sin(t)
    phi = phiRef.sin(t)
    tau = tauRef.sawtooth(t)

    # update animation
    state = np.array([[theta], [phi], [0.0], [0.0]])
    animation.update(state)
    dataPlot.update(t, r, state, tau)

    # advance time by t_plot
    t = t + P.t_plot  
    plt.pause(0.1)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
