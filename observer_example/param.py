import numpy as np
t_start = 0.
t_end = 50.
Ts = 0.01
t_plot = 0.1

# system parameters
A = np.array([[0., 1.], [2., 3.]])
B = np.array([[0.], [4.]])
C = np.array([[1., 0.]])
D = 0.0