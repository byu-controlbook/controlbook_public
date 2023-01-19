import matplotlib.pyplot as plt
import armParam as P
from signalGenerator import signalGenerator
from armAnimation import armAnimation
from dataPlotter import dataPlotter
from armDynamics import armDynamics

# instantiate arm, controller, and reference classes
arm = armDynamics()
#second_arm = armDynamics()
reference = signalGenerator(amplitude=0.01, frequency=0.02)
torque = signalGenerator(amplitude=0.2, frequency=0.05)
#torque2 = signalGenerator(amplitude=0.1, frequency=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
#dataPlot2 = dataPlotter()
animation = armAnimation()
#animation2 = armAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:  
        # Get referenced inputs from signal generators
        r = reference.square(t)
        u = torque.square(t)
        #u2 = torque2.sin(t)
        y = arm.update(u)  # Propagate the dynamics
        #y2 = second_arm.update(u2)
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(arm.state)
    #animation2.update(second_arm.state)
    dataPlot.update(t, r, arm.state, u)
    #dataPlot2.update(t, r, second_arm.state, u2)

    # the pause causes the figure to be displayed during the
    # simulation
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
