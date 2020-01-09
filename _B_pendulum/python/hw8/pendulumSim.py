import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import pendulumParam as P
from hw3.pendulumDynamics import pendulumDynamics
from pendulumController import pendulumController
from hw2.signalGenerator import signalGenerator
from hw2.pendulumAnimation import pendulumAnimation
from hw2.dataPlotter import dataPlotter

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics()
controller = pendulumController()
reference = signalGenerator(amplitude=0.5, frequency=0.05)
disturbance = signalGenerator(amplitude=0)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
y = pendulum.h()  # output of system at start of simulation
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        r = reference.square(t)  # reference input
        d = disturbance.step(t)  # input disturbance
        n = 0.0  #noise.random(t)  # simulate sensor noise
        x = pendulum.state  # use state instead of output
        u = controller.update(r, x)  # update controller
        y = pendulum.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(pendulum.state)
    dataPlot.update(t, r, pendulum.state, u)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
