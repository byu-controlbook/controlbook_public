# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P
#from control.matlab import *
from control import bode
from control import tf
import matplotlib.pyplot as plt


# Compute inner and outer open-loop transfer functions
P_in = tf([-1/(P.m1*P.ell/6+P.m2*2*P.ell/3)],
          [1, 0, -(P.m1+P.m2)*P.g/(P.m1*P.ell/6+P.m2*2*P.ell/3)])
P_out = tf([P.ell/2,0,-P.g], [1, 0, 0])

# Plot the closed loop and open loop bode plots for the inner loop
plt.figure(1), bode(P_in, dB=True)
plt.figure(2), bode(P_out, dB=True)

# Closes plot windows when the user presses a button.
plt.pause(0.0001)
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
