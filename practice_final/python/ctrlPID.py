import numpy as np
import rodMassParam as P


class ctrlPID:
    def __init__(self):

    def update(self, theta_r, y):
        return tau

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







