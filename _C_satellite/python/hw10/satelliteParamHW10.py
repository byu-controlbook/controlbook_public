# satellite Parameter File
import numpy as np
# import control as cnt
import sys
sys.path.append('..')  # add parent directory
sys.path.append('../hw8')
import satelliteParam as P
import satelliteParamHW8 as P8

# import variables from satelliteParam
Ts = P.Ts
sigma = P.sigma
beta = P.beta
tau_max = P.tau_max

####################################################
#       PD Control: Time Design Strategy
####################################################
# PD design for inner loop
kp_th = P8.kp_th
kd_th = P8.kd_th

# DC gain for inner loop
k_DC_th = P8.k_DC_th

# PID design for outer loop
# (we use the PD gains from HW 8, but this is not required)
ki_phi = 0.25  # integral gain for outer loop
kp_phi = P8.kp_phi
kd_phi = P8.kd_phi
theta_max = P8.theta_max

# DC gain for outer loop
k_DC_phi = P8.k_DC_phi

print('k_DC_phi', k_DC_phi)
print('kp_th: ', kp_th)
print('kd_th: ', kd_th)
print('kp_phi: ', kp_phi)
print('kd_phi: ', kd_phi)


