# Single link arm Parameter File
import numpy as np
import armParam as P

Ts = P.Ts  # sample rate of the controller
tau_max = P.tau_max  # limit on control signal

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain

# PD gains
kp = 0.18
kd = 0.095

print('kp: ', kp)
print('kd: ', kd)



