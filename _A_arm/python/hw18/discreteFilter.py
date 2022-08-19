import numpy as np
from control import tf, c2d

class discreteFilter:
    def __init__(self, num, den, Ts):
        self.Ts = Ts
        sys = tf(num, den)
        sys_d = c2d(sys, Ts, method='tustin')
        self.den_d = sys_d.den[0][0]
        self.num_d = sys_d.num[0][0]
        self.prev_filt_output = np.zeros(len(self.num_d)-1)
        self.prev_filt_input = np.zeros(len(self.den_d))

    def update(self, u):
        '''
            Discrete filter implementation for loopshaping controllers
        '''

        # update vector with filter inputs (u)
        self.prev_filt_input = np.hstack(([u], self.prev_filt_input[0:-1]))

        # use filter coefficients to calculate new output (y)
        y = self.num_d @ self.prev_filt_input - self.den_d[1:] @ self.prev_filt_output

        # update vector with filter outputs
        self.prev_filt_output = np.hstack(([y], self.prev_filt_output[0:-1]))

        return y








