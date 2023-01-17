import numpy as np
import armParam as P
from armDynamics import armDynamics
import matplotlib.pyplot as plt 

arm = armDynamics()

with open('io_data.npy', 'rb') as f:
    time_history = np.load(f)
    theta_history = np.load(f)
    torque_history = np.load(f)

N = time_history.shape[0]
total_squared_error = 0
new_theta_history = []

for i in range(0, N):
    #theta = (180 / np.pi ) * arm.update(torque_history[i])
    theta = (180 / np.pi ) * arm.update(0)
    print(torque_history[i])
    new_theta_history.append(theta[0][0])
    total_squared_error += (theta[0][0] - theta_history[i])**2

print('total squared error = ', total_squared_error)

plt.figure(1)
plt.plot(time_history, torque_history)
plt.figure(2)
plt.plot(time_history, theta_history, 'b')
plt.plot(time_history, new_theta_history, 'r')
plt.show()
