# Single link arm Parameter File
import numpy as np
import armParam as P

Ts = P.Ts  # sample rate of the controller
tau_max = P.tau_max  # limit on control signal

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain

#  tuning parameters
tr = 0.8 # part (a)
#tr = 0.4  # tuned to get fastest possible rise time before saturation.
zeta = 0.707

# desired natural frequency and closed-loop coefficients
wn = 2.2/tr
alpha1 = 2.0*zeta*wn
alpha0 = wn**2

# coefficients from original open-loop characteristic equation
b0 = 3.0/(P.m*P.ell**2)
a0 = 0.0
a1 = 3.0*P.b/(P.m*P.ell**2)

# compute PD gains
kp = (alpha0-a0)/b0
kd = (alpha1-a1)/b0

print('kp: ', kp)
print('kd: ', kd)



