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

####################################################
#       PD Control: Time Design Strategy
####################################################
# tuning parameters
tr_th = 0.35  # Rise time for inner loop (theta)
zeta_th = 0.707  # Damping Coefficient for inner loop (theta)
M = 10.0  # Time scale separation between inner and outer loop
zeta_z = 0.707  # Damping Coefficient fop outer loop (z)
ki_z = -0.005  # select integrator gain

# saturation limits
F_max = 5             		  # Max Force, N
error_max = 1        		  # Max step size,m
theta_max = 30.0*np.pi/180.0  # Max theta, rads

# ---------------------------------------------------
#                    Inner Loop
# ---------------------------------------------------
# parameters of the open loop transfer function
b0_th = -1.0/(P.m1*(P.ell/6.0) + P.m2*(2.0*P.ell/3.0))
a1_th = 0.0
a0_th = -(P.m1+P.m2)*P.g/(P.m1*(P.ell/6.0) + P.m2*(2.0*P.ell/3.0))

# coefficients for desired inner loop
# Delta_des(s) = s^2 + alpha1*s + alpha0 = s^2 + 2*zeta*wn*s + wn^2
wn_th = 2.2/tr_th     # Natural frequency
alpha1_th = 2.0*zeta_th*wn_th
alpha0_th = wn_th**2

# compute gains
# Delta(s) = s^2 + (a1 + b0*kd)*s + (a0 + b0*kp)
kp_th = (alpha0_th-a0_th)/b0_th
kd_th = (alpha1_th-a1_th)/b0_th
DC_gain = kp_th/((P.m1+P.m2)*P.g+kp_th)

# ---------------------------------------------------
#                    Outer Loop
# ---------------------------------------------------
# coefficients for desired outer loop
# Delta_des(s) = s^2 + alpha1*s + alpha0 = s^2 + 2*zeta*wn*s + wn^2
tr_z = M*tr_th  # desired rise time, s
wn_z = 2.2/tr_z  # desired natural frequency

# compute gains
a  = -(wn_z**2)*np.sqrt(2.0*P.ell/(3.0*P.g))
b = (a - 2.0*zeta_z*wn_z)*np.sqrt(2.0*P.ell/(3.0*P.g))
kd_z = b/(1-b)
kp_z = a*(1+kd_z)

print('DC_gain', DC_gain)
print('kp_th: ', kp_th)
print('kd_th: ', kd_th)
print('kp_z: ', kp_z)
print('ki_z: ', ki_z)
print('kd_z: ', kd_z)



