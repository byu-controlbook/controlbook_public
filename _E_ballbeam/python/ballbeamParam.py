# Ball on Beam Parameter File
import numpy as np
# import control as cnt

# Physical parameters of the  ballbeam known to the controller
m1 =   # Mass of the ball, kg
m2 =   # mass of beam, kg
length =   # length of beam, m
g =   # gravity at sea level, m/s^2

# parameters for animation
radius = 0.05  # radius of ball

# Initial Conditions
z0 =  # initial ball position,m
theta0 = *np.pi/180  # initial beam angle,rads
zdot0 =   # initial speed of ball along beam, m/s
thetadot0 =  # initial angular speed of theh beam,rads/s

# Simulation Parameters
t_start =   # Start time of simulation
t_end =   # End time of simulation
Ts =   # sample time for simulation
t_plot =   # the plotting and animation is updated at this rate

# saturation limits
Fmax =   # Max Force, N

# dirty derivative parameters
# sigma =   # cutoff freq for dirty derivative
# beta =  # dirty derivative gain

# equilibrium force when ball is in center of beam
# ze =
# Fe =
