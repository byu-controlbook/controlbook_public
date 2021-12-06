# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P
import numpy as np
from scipy import signal
import control as cnt

# sample rate of the controller
Ts = P.Ts

# saturation limits
F_max = P.F_max  # Max Force, N

# tuning parameters
tr_z = 1.5  # rise time for position
tr_theta = 0.5  # rise time for angle
zeta_z = 0.707  # damping ratio position
zeta_th = 0.707  # damping ratio angle
integrator_pole = -10.0  # integrator pole
tr_z_obs = tr_z/10.0  # rise time for observer - position
tr_theta_obs = tr_theta/10.0  # rise time for observer - angle
dist_obsv_pole = -1.0  # pole for disturbance observer

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.array([[0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0],
               [0.0, -3*P.m1*P.g/4/(.25*P.m1+P.m2),
                -P.b/(.25*P.m1+P.m2), 0.0],
               [0.0, 3*(P.m1+P.m2)*P.g/2/(.25*P.m1+P.m2)/P.ell,
                3*P.b/2/(.25*P.m1+P.m2)/P.ell, 0.0]])
B = np.array([[0.0],
               [0.0],
               [1/(.25*P.m1+P.m2)],
               [-3.0/2/(.25*P.m1+P.m2)/P.ell]])
C = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])

# form augmented system
Cr = np.array([[1.0, 0.0, 0.0, 0.0]])

# Augmented Matrices
A1 = np.concatenate((
        np.concatenate((A, np.zeros((4, 1))), axis=1),
        np.concatenate((-Cr, np.array([[0.0]])), axis=1)),
        axis=0)
B1 = np.concatenate((B, np.array([[0.0]])), axis=0)

# computer control gains
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly = np.convolve(
    np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                [1, 2*zeta_th*wn_th, wn_th**2]),
    np.poly([integrator_pole]))
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
    print("The system is not controllable")
else:
    K1 = cnt.acker(A1, B1, des_poles)
    K = K1[0, 0:4]
    ki = K1[0, 4]

# compute observer gains
# Augmented Matrices
A2 = np.concatenate((
        np.concatenate((A, B), axis=1),
        np.zeros((1, 5))),
        axis=0)
C2 = np.concatenate((C, np.zeros((2, 1))), axis=1)

wn_z_obs = 2.2/tr_z_obs
wn_th_obs = 2.2/tr_theta_obs
des_obs_char_poly = np.convolve(
    np.convolve([1, 2*zeta_z*wn_z_obs, wn_z_obs**2],
                [1, 2*zeta_th*wn_th_obs, wn_th_obs**2]),
    np.poly([dist_obsv_pole]))
des_obs_poles = np.roots(des_obs_char_poly)

# Compute the gains if the system is observable
if np.linalg.matrix_rank(cnt.ctrb(A2.T, C2.T)) != 5:
    print("The system is not observable")
else:
     L2 = signal.place_poles(A2.T, C2.T, des_obs_poles).gain_matrix.T

print('K: ', K)
print('ki: ', ki)
print('L^T: ', L2.T)

