# Inverted Pendulum Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P

# sample rate of the controller
Ts = P.Ts

# saturation limits
F_max = 5.0                # Max Force, N

####################################################
#                 State Space
####################################################
# tuning parameters
tr_z = 1.0        # rise time for position
tr_theta = 0.5    # rise time for angle
zeta_z   = 0.707  # damping ratio position
zeta_th  = 0.707  # damping ratio angle

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

# gain calculation
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly = np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                            [1, 2*zeta_th*wn_th, wn_th**2])
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 4:
    print("The system is not controllable")
else:
    K = cnt.acker(A, B, des_poles)
    Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
    kr = -1.0/(Cr*np.linalg.inv(A-B@K)@B)

print('K: ', K)
print('kr: ', kr)




