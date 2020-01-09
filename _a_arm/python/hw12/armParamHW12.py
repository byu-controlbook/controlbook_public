# Single link arm Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import armParam as P

Ts = P.Ts  # sample rate of the controller
beta = P.beta  # dirty derivative gain
tau_max = P.tau_max  # limit on control signal
m = P.m
ell = P.ell
g = P.g

#  tuning parameters
tr = 0.4
zeta = 0.707
integrator_pole = -5

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.array([[0.0, 1.0],
              [0.0, -1.0*P.b/P.m/(P.ell**2)]])

B = np.array([[0.0],
              [3.0/P.m/(P.ell**2)]])

C = np.array([[1.0, 0.0]])

# form augmented system
A1 = np.array([[0.0, 1.0, 0.0],
               [0.0, -1.0*P.b/P.m/(P.ell**2), 0.0],
               [-1.0, 0.0, 0.0]])

B1 = np.array([[0.0],
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
    K = np.array([[K1.item(0), K1.item(1)]])
    ki = K1.item(2)

print('K: ', K)
print('ki: ', ki)



