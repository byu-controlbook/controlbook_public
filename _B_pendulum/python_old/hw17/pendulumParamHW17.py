# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P
sys.path.append('../hw10')  # add parent directory
import pendulumParamHW10 as P10
# import numpy as np
# from scipy import signal
import control as cnt
from control import TransferFunction as tf
import matplotlib.pyplot as plt


# Compute inner and outer open-loop transfer functions
temp = P.m1*P.ell/6+P.m2*2*P.ell/3;
P_in = tf([-1/temp],[1,0,-(P.m1+P.m2)*P.g/temp]);
P_out = tf([-2*P.ell/3,0,P.g],[1,0,0]);

# Compute inner and outer closed-loop transfer functions
C_in = tf([P10.kd_th+P10.sigma*P10.kp_th, P10.kp_th], [P10.sigma, 1])
C_out = tf([P10.kd_z+P10.kp_z*P10.sigma, P10.kp_z+P10.ki_z*P10.sigma, P10.ki_z], [P10.sigma, 1, 0])


# Plot the closed loop and open loop bode plots for the inner loop
plt.figure(1), plt.clf(), plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_in*C_in, dB=True)
cnt.bode(P_in*C_in/(1+P_in*C_in), dB = True)

# Plot the closed loop and open loop bode plots for the outer loop
plt.figure(2), plt.clf(), plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_out*C_out, dB=True)
cnt.bode(P_out*C_out/(1+P_out*C_out), dB = True)

# Calculate the phase and gain margin
gm, pm, Wcg, Wcp = cnt.margin(P_in*C_in)
print("gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

gm, pm, Wcg, Wcp = cnt.margin(P_out*C_out)
print("gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)



# # display bode plots of transfer functions
# plt.figure(1), plt.clf, plt.hold(True), plt.grid(True)
# cnt.matlab.bode(P_in, P_in*C_in, dB=True)
# #plt.legend('No control', 'PD')
# plt.title('Inverted Pendulum, Inner Loop')
#
# plt.figure(2), plt.clf, plt.hold(True), plt.grid(True)
# cnt.matlab.bode(P_out, P_out*C_out, tf([1.0], [1.0, 0.0]), dB=True)
# #legend('No control', 'PID','1/s')
# plt.title('Inverted Pendulum, Outer Loop')


# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
