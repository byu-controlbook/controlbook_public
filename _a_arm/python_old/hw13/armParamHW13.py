# Single link arm Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import armParam as P

#  tuning parameters
tr = 0.4
zeta = 0.707
integrator_pole = -5
wn_obs = 10  # natural frequency for observer
zeta_obs = 0.707  # damping ratio for observer

Ts = P.Ts  # sample rate of the controller
tau_max = P.tau_max  # limit on control signal
m = P.m
ell = P.ell
g = P.g


# State Space Equations
A = np.matrix([[0.0, 1.0],
               [0.0, -1.0*P.b/P.m/(P.ell**2)]])

B = np.matrix([[0.0],
               [3.0/P.m/(P.ell**2)]])

C = np.matrix([[1.0, 0.0]])

# control design
# form augmented system
A1 = np.matrix([[0.0, 1.0, 0.0],
               [0.0, -1.0*P.b/P.m/(P.ell**2), 0.0],
               [-1.0, 0.0, 0.0]])

B1 = np.matrix([[0.0],
               [3.0/P.m/(P.ell**2)],
               [0.0]])

# gain calculation
wn = 2.2/tr  # natural frequency
des_char_poly = np.convolve(
    [1, 2*zeta*wn, wn**2],
    np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
    print("The system is not controllable")
else:
    K1 = cnt.acker(A1, B1, des_poles)
    K = np.matrix([K1.item(0), K1.item(1)])
    ki = K1.item(2)

# observer design
des_obsv_char_poly = [1, 2*zeta_obs*wn_obs, wn_obs**2]
des_obsv_poles = np.roots(des_obsv_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 2:
    print("The system is not observerable")
else:
    L = cnt.acker(A.T, C.T, des_obsv_poles).T

print('K: ', K)
print('ki: ', ki)
print('L^T: ', L.T)



