# satellite Parameter File
import numpy as np
import control as cnt
from scipy import signal
import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P

Ts = P.Ts
sigma = P.sigma
beta = P.beta
tau_max = P.tau_max

# tuning parameters
wn_th = 0.6
wn_phi = 1.1    # rise time for angle
zeta_phi = 0.707  # damping ratio position
zeta_th = 0.707  # damping ratio angle
integrator_pole = np.array([-1.0])
# pick observer poles
wn_th_obs = 10.0*wn_th
wn_phi_obs = 10.0*wn_phi
dist_obsv_pole = np.array([-1.0])  # disturbance observer pole

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.array([[0.0, 0.0,               1.0,      0.0],
               [0.0, 0.0,               0.0,      1.0],
               [-P.k/P.Js, P.k/P.Js, -P.b/P.Js, P.b/P.Js],
               [P.k/P.Jp, -P.k/P.Jp, P.b/P.Jp, -P.b/P.Jp]])

B = np.array([[0.0],
               [0.0],
               [1.0/P.Js],
               [0.0]])

C = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])

# form augmented system
Cout = np.array([[0.0, 1.0, 0.0, 0.0]])
A1 = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 1.0, 0.0],
               [-P.k/P.Js, P.k/P.Js, -P.b/P.Js, P.b/P.Js, 0.0],
               [P.k/P.Jp, -P.k/P.Jp, P.b/P.Jp, -P.b/P.Jp, 0.0],
               [0.0, -1.0, 0.0, 0.0, 0.0]])

B1 = np.array([[0.0],
               [0.0],
               [1.0/P.Js],
               [0.0],
               [0.0]])


# gain calculation
des_char_poly = np.convolve(
    np.convolve([1, 2*zeta_phi*wn_phi, wn_phi**2],
                [1, 2*zeta_th*wn_th, wn_th**2]),
    np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
    print("The system is not controllable")
else:
    K1 = cnt.acker(A1, B1, des_poles)
    K = np.array([K1.item(0),
                  K1.item(1),
                  K1.item(2),
                  K1.item(3)])
    ki = K1.item(4)

# compute observer gains
# Augmented Matrices
A2 = np.concatenate((
        np.concatenate((A, B), axis=1),
        np.zeros((1, 5))),
        axis=0)
C2 = np.concatenate((C, np.zeros((2, 1))), axis=1)
des_obs_char_poly = np.convolve(
    np.convolve([1, 2*zeta_phi*wn_phi_obs, wn_phi_obs**2],
                [1, 2*zeta_th*wn_th_obs, wn_th_obs**2]),
    np.poly(dist_obsv_pole))
des_obs_poles = np.roots(des_obs_char_poly)

# Compute the gains if the system is observable
if np.linalg.matrix_rank(cnt.ctrb(A2.T, C2.T)) != 5:
    print("The system is not observable")
else:
    # place_poles returns an object with various properties.
    # The gains are accessed through .gain_matrix
    # .T transposes the matrix
    L2 = signal.place_poles(A2.T,
                            C2.T,
                            des_obs_poles).gain_matrix.T
    L = L2[0:4, 0:2]
    Ld = L2[4:5, 0:2]

print('K: ', K)
print('ki: ', ki)
print('L^T: ', L.T)
print('Ld: ', Ld)



