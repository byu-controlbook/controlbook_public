# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import armParam as P
sys.path.append('../hw10')  # add parent directory
import armParamHW10 as P10
import numpy as np
# from scipy import signal
from control.matlab import *
from control import TransferFunction as tf
import matplotlib.pyplot as plt

# Compute plant transfer functions
th_e = 0
Plant = tf([2.0/P.m/P.ell**2],
           [1, 2.0*P.b/P.m/P.ell**2, -3.0*P.g*np.sin(th_e)/2/P.ell])

# Compute transfer function of controller
C_pid = tf([(P10.kd+P10.kp*P.sigma), (P10.kp+P10.ki*P.sigma), P10.ki],
           [P.sigma, 1, 0])


# display bode plots of transfer functions
plt.figure(3),
bode(Plant, Plant*C_pid, dB=True)
#plt.legend('No control', 'PID')
plt.title('Single Link Arm')



# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
