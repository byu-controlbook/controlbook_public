import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from signalGenerator import SignalGenerator
from hummingbirdAnimation import HummingbirdAnimation
from dataPlotter import DataPlotter

# instantiate reference input classes
phi_ref = SignalGenerator(amplitude=1.5, frequency=0.05)
theta_ref = SignalGenerator(amplitude=0.5, frequency=0.05)
psi_ref = SignalGenerator(amplitude=0.5, frequency=.05)

# instantiate the simulation plots and animation
dataPlot = DataPlotter()
animation = HummingbirdAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    phi = phi_ref.sin(t)
    theta = 0  # theta_ref.sin(t)
    psi = 0  # psi_ref.sin(t)
    # update animation
    state = np.array([[phi], [theta], [psi], [0.0], [0.0], [0.0]])
    ref = np.array([[0], [0], [0]])
    fl = 0
    fr = 0
    pwm = np.array([[fl], [fr]])
    animation.update(t, state)
    dataPlot.update(t, state, ref, pwm)

    t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.05)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
