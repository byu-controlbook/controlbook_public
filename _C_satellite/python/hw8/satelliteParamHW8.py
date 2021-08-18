# satellite Parameter File
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P

# import variables from satelliteParam for later import through current file
Ts = P.Ts
sigma = P.sigma
beta = P.beta
tau_max = P.tau_max

####################################################
#       PD Control: Time Design Strategy
####################################################
# PD design for inner loop
zeta_th = 0.9  # damping ratio for inner loop
theta_max = 30.0*np.pi/180.0  # maximum commanded base angle
tr_th = 1.0
wn_th = 0.5 * np.pi / (tr_th * np.sqrt(1 - zeta_th**2))# 2.2/tr_th
kp_th = wn_th**2 * (P.Js + P.Jp)
kd_th = 2*zeta_th * wn_th * (P.Js + P.Jp)

# DC gain for inner loop
k_DC_th = kp_th/(P.k+kp_th)

# PD design for outer loop
# tuning parameters
M = 10.0  # time scale separation between inner and outer loop
tr_phi = M*tr_th  # rise time for outer loop
zeta_phi = 0.9  # damping ratio for outer loop
wn_phi = 0.5 * np.pi / (tr_phi * np.sqrt(1 - zeta_phi**2)) #2.2/tr_phi

AA = np.array([
    [P.k*k_DC_th, -P.b*k_DC_th*wn_phi**2],
    [P.b*k_DC_th, P.k*k_DC_th-2*zeta_phi*wn_phi*P.b*k_DC_th]])

bb = np.array([
    [-P.k+P.Jp*wn_phi**2],
    [-P.b+2*P.Jp*zeta_phi*wn_phi]])
tmp = np.linalg.inv(AA) @ bb

kp_phi = tmp.item(0)
kd_phi = tmp.item(1)

# DC gain for outer loop
k_DC_phi = P.k*k_DC_th*kp_phi/(P.k+P.k*k_DC_th*kp_phi)

print('k_DC_phi', k_DC_phi)
print('kp_th: ', kp_th)
print('kd_th: ', kd_th)
print('kp_phi: ', kp_phi)
print('kd_phi: ', kd_phi)
print('P.k is ', P.k)

# Used to check the math in the matrix inversion.
# print(np.roots([1.0, (P.b+P.b*k_DC_th*kp_phi+P.k*k_DC_th*kd_phi)/(P.Jp + P.b*k_DC_th*kd_phi),\
#                 (P.k + P.k*k_DC_th*kp_phi)/(P.Jp + P.b*k_DC_th*kd_phi)]))
#
# print(np.roots([1.0, 2*zeta_phi*wn_phi, wn_phi**2]))
