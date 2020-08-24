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

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.array([[0.0, 1.0],
               [0.0, -1.0*P.b/P.m/(P.ell**2)]])

B = np.array([[0.0],
               [3.0/P.m/(P.ell**2)]])

C = np.array([[1.0, 0.0]])

# gain calculation
wn = 2.2/tr  # natural frequency
des_char_poly = [1, 2*zeta*wn, wn**2]
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 2:
    print("The system is not controllable")

else:
    #.A just turns K matrix into a numpy array
    K = (cnt.acker(A, B, des_poles)).A 
    kr = -1.0/(C @ np.linalg.inv(A - B @ K) @ B)

print('K: ', K)
print('kr: ', kr)



