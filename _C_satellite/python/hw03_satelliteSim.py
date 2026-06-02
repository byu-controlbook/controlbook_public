import matplotlib.pyplot as plt
import satelliteParam as P
from signalGenerator import signalGenerator
from satelliteAnimation import satelliteAnimation
from dataPlotter import dataPlotter
from satelliteDynamics import satelliteDynamics

# instantiate satellite, controller, and reference classes
satellite = satelliteDynamics(alpha=0.0)
torque = signalGenerator(amplitude=0.1, frequency=0.1)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = satelliteAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:  
        u = torque.sin(t)
        y = satellite.update(u)  # Propagate the dynamics
        t += P.Ts  # advance time by Ts
        
    # update animation and data plots
    animation.update(satellite.state)
    dataPlot.update(t, satellite.state, u)
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
