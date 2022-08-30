# Inverted Pendulum Parameter File
import numpy as np
import control as cnt
import pendulumParam as P

# sample rate of the controller
Ts = P.Ts

# saturation limits
F_max = 5.0                # Max Force, N
theta_max = 30.0*np.pi/180.0  # Max theta, rads

####################################################
#                 State Space
####################################################
# tuning parameters
tr_z = 1.5             # rise time for position
tr_theta = 0.5         # rise time for angle
zeta_z = 0.707       # damping ratio position
zeta_th = 0.707       # damping ratio angle
integrator_pole = -2  # integrator pole

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
Cout = np.array([[1.0, 0.0, 0.0, 0.0]])
A1 = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 1.0, 0.0],
               [0.0, -3*P.m1*P.g/4/(.25*P.m1+P.m2),
                -P.b/(.25*P.m1+P.m2), 0.0, 0.0],
               [0.0, 3*(P.m1+P.m2)*P.g/2/(.25*P.m1+P.m2)/P.ell,
                3*P.b/2/(.25*P.m1+P.m2)/P.ell, 0.0, 0.0],
               [-1.0, 0.0, 0.0, 0.0, 0.0]])
B1 = np.array([[0.0],
               [0.0],
               [1/(.25*P.m1+P.m2)],
               [-3.0/2/(.25*P.m1+P.m2)/P.ell],
               [0.0]])

# gain calculation
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
    K = K1[0, 0:4].reshape(1,4)
    ki = K1[0, 4].reshape(1,1)

print('K: ', K)
print('ki: ', ki)




