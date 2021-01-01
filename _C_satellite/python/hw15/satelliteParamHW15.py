# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P
from control.matlab import *
from control import TransferFunction as tf
import matplotlib.pyplot as plt


# Compute inner and outer open-loop transfer functions
#P_in = tf([1/P.Js], [1, P.b/P.Js, P.k/P.Js])
P_in = tf([1],[P.Js+P.Jp,0,0]);
P_out = tf([P.b/P.Jp, P.k/P.Jp], [1, P.b/P.Jp, P.k/P.Jp])


# Plot the closed loop and open loop bode plots for the inner loop
plt.figure(3), bode(P_in, dB=True)
plt.figure(4), bode(P_out, dB=True)

# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
