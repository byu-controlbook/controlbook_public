import numpy as np
import pendulumParam as P
import pendulumParamHW8 as P8


class pendulumController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        # Instantiates the SS_ctrl object
        self.kp_z = P8.kp_z
        self.kd_z = P8.kd_z
        self.kp_th = P8.kp_th
        self.kd_th = P8.kd_th
        self.filter = zeroCancelingFilter()

    def update(self, y_r, state):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)
        # the reference angle for theta comes from the outer loop PD control
        tmp = self.kp_z * (z_r - z) - self.kd_z * zdot
        theta_r = self.filter.update(tmp)
        # the force applied to the cart comes from the inner loop PD control
        F = self.kp_th * (theta_r - theta) - self.kd_th * thetadot
        return F

class zeroCancelingFilter:
    def __init__(self):
        self.a = -3.0/(2.0*P.ell*P8.DC_gain)
        self.b = np.sqrt(3.0*P.g/(2.0*P.ell))
        self.w = 0.0

    def update(self, v):
        self.w = self.w + P.Ts * (-self.b * self.w + self.a * v)
        return self.w






