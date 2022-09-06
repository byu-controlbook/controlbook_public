# mass-spring-damper Parameter File
import numpy as np

# Physical parameters of the arm known to the controller
m =   # mass kg
k =   # spring constant Kg/s^2
b =   # damping coefficient Kg/s

# parameters for animation
length = 5.0
width = 1.0

# Initial Conditions
z0 =   # initial position of mass, m
zdot0 =   # initial velocity of mass m/s

# Simulation Parameters
t_start =  # Start time of simulation
t_end =   # End time of simulation
Ts =   # sample time for simulation
t_plot =  # the plotting and animation is updated at this rate

# dirty derivative parameters
# sigma =  # cutoff freq for dirty derivative
# beta =   # dirty derivative gain

# saturation limits
# F_max =   # Max force, N

