# Hummingbird Parameter File
import numpy as np
# Initial Conditions
phi0 = 0.0 * np.pi / 180  # roll angle in rads
theta0 = 0 * np.pi / 180  # pitch angle in rads
psi0 = 0.0 * np.pi / 180  # yaw angle in rads
phidot0 = 0.0              # roll rate in rads/sec
thetadot0 = 0.0         # pitch rate in rads/sec
psidot0 = 0.0              # yaw rate in rads/sec
# Physical parameters of the hummingbird known to the controller
g = 
ell1 = 
ell2 = 
ell3x = 
ell3y = 
ell3z = 
ellT = 
d = 
m1 = 
J1x = 
J1y = 
J1z = 
m2 = 
J2x = 
J2y = 
J2z = 
m3 = 
J3x = 
J3y = 
J3z = 
km = g * (m1 * ell1 + m2 * ell2) / ellT  # need to find this experimentally for hardware

# mixing matrix
unmixing = np.array([[1.0, 1.0], [d, -d]]) # converts fl and fr (LR) to force and torque (FT)
mixing = np.linalg.inv(unmixing) # converts force and torque (FT) to fl and fr (LR) 

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 100.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate
# saturation limits
force_max = 2.0                # Max force N
torque_max = 5.0                # Max torque, Nm

