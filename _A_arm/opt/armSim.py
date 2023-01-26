import matplotlib.pyplot as plt
import numpy as np
import armParam as P
from signalGenerator import signalGenerator
from armAnimation import armAnimation
from dataPlotter import dataPlotter
from armDynamics import armDynamics

def armSim(controller):
    # instantiate arm, and reference classes
    arm = armDynamics(alpha=0.2)
    reference = signalGenerator(amplitude=30*np.pi/180.0, frequency=0.05)
    disturbance = signalGenerator(amplitude=0.1)
    # instantiate the simulation plots and animation
    dataPlot = dataPlotter()
    animation = armAnimation()
    # simulation starts at time t_start
    t = P.t_start  
    # output of system at start of simulation
    y = arm.h()  
    #-----------------------------------
    # main simulation loop  
    while t < P.t_end:  
        # plot is updated every P.t_plot second
        t_next_plot = t + P.t_plot
        # updates control and dynamics at faster simulation rate P.Ts
        while t < t_next_plot: 
            r = reference.square(t)  # reference input
            d = disturbance.step(t)  # input disturbance
            n = 0.0  #noise.random(t)  # sensor noise
            u = controller.update(r, y + n)  # update controller
            y = arm.update(u + d)  # propagate system
            t = t + P.Ts  # advance time by Ts
        # update animation and data plots
        animation.update(arm.state)
        dataPlot.update(t, r, arm.state, u)
        # the pause causes the figure to display during simulation
        plt.pause(0.0001)  
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()
