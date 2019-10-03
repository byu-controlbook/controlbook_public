import pendulumParamHW8 as P
from PDControl import PDControl

class pendulumController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        # Instantiates the SS_ctrl object
        self.kp_z = P.kp_z
        self.kd_z = P.kd_z
        self.kp_th = P.kp_th
        self.kd_th = P.kd_th

    def update(self, y_r, state):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)
        # the reference angle for theta comes from the outer loop PD control
        theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot
        # the force applied to the cart comes from the inner loop PD control
        F = self.kp_th * (theta_r - theta) - self.kd_th * thetadot
        return F







