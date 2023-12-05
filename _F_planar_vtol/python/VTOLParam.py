# VTOL Parameter File
import numpy as np

# Physical parameters of the  VTOL known to the controller
mc =  # kg
mr =   # kg
Jc =   # kg m^2
d =   # m
mu =   # kg/s
g =   # m/s^2
F_wind =  # wind disturbance force is zero in initial homeworks

# parameters for animation
length = 10.0

# Initial Conditions
z0 =   # initial lateral position
h0 =   # initial altitude
theta0 =  # initial roll angle
zdot0 =   # initial lateral velocity
hdot0 =   # initial climb rate
thetadot0 =   # initial roll rate
target0 =

# Simulation Parameters
t_start =  # Start time of simulation
t_end =   # End time of simulation
Ts =   # sample time for simulation
t_plot =  # the plotting and animation is updated at this rate

# saturation limits
fmax =   # Max Force, N

# dirty derivative parameters
# sigma =   # cutoff freq for dirty derivative
# beta =  # dirty derivative gain

# equilibrium force
# Fe =

# mixing matrix
unmixing = np.array([[1.0, 1.0], [d, -d]]) # converts fl and fr (LR) to force and torque (FT)
mixing = np.linalg.inv(unmixing) # converts force and torque (FT) to fl and fr (LR) 

