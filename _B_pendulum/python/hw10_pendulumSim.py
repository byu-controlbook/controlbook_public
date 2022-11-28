import matplotlib.pyplot as plt
import pendulumParam as P
from hw2.signalGenerator import signalGenerator
from hw2.pendulumAnimation import pendulumAnimation
from hw2.dataPlotter import dataPlotter
from hw3.pendulumDynamics import pendulumDynamics
from hw10.pendulumController import pendulumController

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics(alpha=0.2)
controller = pendulumController()
reference = signalGenerator(amplitude=0.5, frequency=0.06)
disturbance = signalGenerator(amplitude=0.1)
noise = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
y = pendulum.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.square(t)  # reference input
        d = disturbance.step(t)  # input disturbance
        n = 0.0 #noise.random(t)  # simulate sensor noise
        u = controller.update(r, y + n)  # update controller
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
