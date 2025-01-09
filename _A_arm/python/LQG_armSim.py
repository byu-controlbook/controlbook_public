import matplotlib.pyplot as plt
import numpy as np
import armParam as P
from signalGenerator import signalGenerator
from armAnimation import armAnimation
from dataPlotter import dataPlotter
from armDynamics import armDynamics
from ctrlLQR import LQR
from ctrlKalmanFilter import KalmanFilter
from dataPlotterObserver import dataPlotterObserver


arm = armDynamics(alpha=0)
controller = LQR()
estimator = KalmanFilter()
reference = signalGenerator(amplitude=np.radians(30), frequency=0.05)

dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = armAnimation()

t = P.t_start
y = arm.h()
u = estimator._get_equilibrium_input(y)
while t < P.t_end:
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        theta_r = reference.square(t)
        x_r = np.array([[theta_r, 0.0]]).T
        xhat = estimator.update(y, u)
        u = controller.update(x_r, xhat)
        y = arm.update(u)
        t += P.Ts

    animation.update(arm.state)
    dataPlot.update(t, arm.state, u, theta_r)
    dataPlotObserver.update(t, arm.state, xhat, 0.0, 0.0)
    plt.pause(0.0001) # the pause causes the figure to display during simulation

# Keeps the program from closing until the user presses a button
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
