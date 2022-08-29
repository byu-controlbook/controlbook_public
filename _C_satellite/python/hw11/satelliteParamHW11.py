# satellite Parameter File
import numpy as np
import control as cnt
import satelliteParam as P

# import variables from satelliteParam
Ts = P.Ts
tau_max = P.tau_max

# tuning parameters
tr_phi = 2
tr_th = 3.66
zeta_phi = 0.707  # damping ratio position
zeta_th = 0.707  # damping ratio angle

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

# gain calculation
wn_th = 2.2/tr_th
wn_phi = 2.2/tr_phi
des_char_poly = np.convolve([1, 2*zeta_th*wn_th, wn_th**2],
                            [1, 2*zeta_phi*wn_phi, wn_phi**2])
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 4:
    print("The system is not controllable")
else:
    K = cnt.acker(A, B, des_poles)
    Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
    kr = -1.0/(Cr @ np.linalg.inv(A - B @ K) @ B)

print('K: ', K)
print('kr: ', kr)



