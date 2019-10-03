# Inverted Pendulum Parameter File
#import numpy as np
#import control as cnt

# Physical parameters of the satellite known to the controller
Js = 5.0      # kg m^2
Jp = 1.0      # kg m^2
k = 0.15      # N m
b = 0.05      # N m s

# parameters for animation
length = 1.0  # length of solar panel
width = 0.3   # width of satellite body

# Initial Conditions
theta0 = 0.0     # initial base angle
phi0 = 0.0       # initial panel angle
thetadot0 = 0.0  # initial angular rate of base
phidot0 = 0.0    # initial angular rate of panel

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 50.0   # End time of simulation
Ts = 0.01      # sample time for simulation
t_plot = 0.1   # the plotting and animation is updated at this rate

# saturation limits
tau_max = 5.0  # Max torque, Nm

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain


