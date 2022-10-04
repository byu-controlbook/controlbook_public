# Single link arm Parameter File
import numpy as np
import armParam as P

Ts = P.Ts  # sample rate of the controller
tau_max = P.tau_max  # limit on control signal

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain

b0 = 3.0/(P.m*P.ell**2)
a1 = 3.0*P.b/(P.m*P.ell**2)
a0 = 0.0 

# select desired closed loop char eq
Delta_cl_d = np.poly([-3, -4])

# PD gains
kp = (Delta_cl_d[2]-a0)/b0
kd = (Delta_cl_d[1]-a1)/b0

print('kp: ', kp)
print('kd: ', kd)
