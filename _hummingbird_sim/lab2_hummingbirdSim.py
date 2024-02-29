import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from signalGenerator import SignalGenerator
from hummingbirdAnimation import HummingbirdAnimation
from dataPlotter import DataPlotter

# instantiate reference input classes
phi_fakeValueGenerator = SignalGenerator(amplitude=1.5, frequency=0.05)
theta_fakeValueGenerator = SignalGenerator(amplitude=0.5, frequency=0.1)
psi_fakeValueGenerator = SignalGenerator(amplitude=0.5, frequency=.15)
force_fakeValueGenerator = SignalGenerator(amplitude=0.5, frequency=.05)
torque_fakeValueGenerator = SignalGenerator(amplitude=0.5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = DataPlotter()
animation = HummingbirdAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    phi = phi_fakeValueGenerator.sin(t)
    theta = theta_fakeValueGenerator.sin(t)
    psi = psi_fakeValueGenerator.sin(t)
    state = np.array([[phi], [theta], [psi], [0.0], [0.0], [0.0]])
    force = force_fakeValueGenerator.sin(t)
    torque = torque_fakeValueGenerator.sin(t)
    # convert force and torque to pwm values
    pwm = P.mixing @ np.array([[force], [torque]]) / P.km
    # update animation
    animation.update(t, state)
    dataPlot.update(t, state, pwm)

    t += P.t_plot  # advance time by t_plot
    plt.pause(0.05)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
