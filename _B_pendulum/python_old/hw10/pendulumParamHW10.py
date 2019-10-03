# Inverted Pendulum Parameter File
import numpy as np
# import control as cnt
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P

# sample rate of the controller
Ts = P.Ts

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2 * sigma - Ts) / (2 * sigma + Ts)  # dirty derivative gain

####################################################
#       PD Control: Time Design Strategy
####################################################
tr_z = 2           # rise time for outer loop   - Can't really push saturation limits and remain stable.
zeta_z   = 0.707   # damping ratio for outer loop
M = 8              # time scale separation between inner and outer loop
zeta_th  = 0.707   # damping ratio for inner loop
ki_z = 0.0001    # select integrator gain


# saturation limits
F_max = 5                   # Max Force, N
theta_max = 30.0*np.pi/180.0   # Max theta, rads

# ---------------------------------------------------
#                    Inner Loop
# ---------------------------------------------------
# gains for inner loop
tr_theta = tr_z/M  # rise time for inner loop
wn_th = 2.2/tr_theta  # natural frequency for inner loop
kp_th = -(P.m1+P.m2)*P.g - P.m2*P.ell/2*wn_th**2  # kp - inner loop
kd_th = -zeta_th*wn_th*P.m2*P.ell  # kd - inner loop
# DC gain for inner loop
DC_gain = kp_th/((P.m1+P.m2)*P.g+kp_th)

# ---------------------------------------------------
#                    Outer Loop
# ---------------------------------------------------

# PID design for outer loop
wn_z = 2.2/tr_z  # natural frequency for outer loop
kp_z = wn_z**2/P.g/DC_gain
kd_z = 2*zeta_z*wn_z/P.g/DC_gain

print('DC_gain', DC_gain)
print('kp_th: ', kp_th)
print('kd_th: ', kd_th)
print('kp_z: ', kp_z)
print('ki_z: ', ki_z)
print('kd_z: ', kd_z)



