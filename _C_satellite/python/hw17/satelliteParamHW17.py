# Satellite Parameter File
import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P
sys.path.append('../hw10')  # add parent directory
import satelliteParamHW10 as P10
# import numpy as np
# from scipy import signal
import control as cnt
from control import TransferFunction as tf
import matplotlib.pyplot as plt


# Compute inner and outer open-loop transfer functions
P_in = tf([1/P.Js], [1, P.b/P.Js, P.k/P.Js])
P_out = tf([P.b/P.Jp, P.k/P.Jp], [1, P.b/P.Jp, P.k/P.Jp])

C_in = tf([(P10.kd_th+P10.sigma*P10.kp_th), P10.kp_th], [P10.sigma, 1])
C_out = tf([(P10.kd_phi+P10.kp_phi*P10.sigma), (P10.kp_phi+P10.ki_phi*P10.sigma), P10.ki_phi], [P10.sigma, 1, 0])

# Plot the closed loop and open loop bode plots for the inner loop
plt.figure(3), plt.clf(), plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_in*C_in, dB=True)
cnt.bode(P_in*C_in/(1+P_in*C_in), dB = True)

# Plot the closed loop and open loop bode plots for the outer loop
plt.figure(4), plt.clf(), plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_out*C_out, dB=True)
cnt.bode(P_out*C_out/(1+P_out*C_out), dB = True)

# Calculate the phase and gain margin
gm, pm, Wcg, Wcp = cnt.margin(P_in*C_in)
print("gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

gm, pm, Wcg, Wcp = cnt.margin(P_out*C_out)
print("gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
