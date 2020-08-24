# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P
sys.path.append('../hw10')  # add parent directory
import pendulumParamHW10 as P10
# import numpy as np
# from scipy import signal
from control.matlab import *
from control import TransferFunction as tf
import matplotlib.pyplot as plt


# Compute inner and outer open-loop transfer functions
temp = P.m1*P.ell/6+P.m2*2*P.ell/3;
P_in = tf([-1/temp],[1,0,-(P.m1+P.m2)*P.g/temp]);
P_out = tf([-2*P.ell/3,0,P.g],[1,0,0]);

# Compute inner and outer closed-loop transfer functions
C_in = tf([P10.kd_th+P10.sigma*P10.kp_th, P10.kp_th], [P10.sigma, 1])
C_out = tf([P10.kd_z+P10.kp_z*P10.sigma, P10.kp_z+P10.ki_z*P10.sigma, P10.ki_z], [P10.sigma, 1, 0])


# display bode plots of transfer functions
plt.figure(1), plt.clf, plt.grid(True)
bode(P_in, P_in*C_in, dB=True)
#plt.legend('No control', 'PD')
plt.title('Inverted Pendulum, Inner Loop')

plt.figure(2), plt.clf, plt.grid(True)
bode(P_out, P_out*C_out, tf([1.0], [1.0, 0.0]), dB=True)
#legend('No control', 'PID','1/s')
plt.title('Inverted Pendulum, Outer Loop')


# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
