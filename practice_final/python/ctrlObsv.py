import numpy as np
import rodMassParam as P
import control as cnt

class ctrlObsv:
    def __init__(self):

    def update(self, theta_r, y):
        return tau, x_hat, d_hat

    def update_observer(self, y):
         return x_hat, d_hat

    def observer_f(self, x_hat, y):
        return xhat_dot

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

