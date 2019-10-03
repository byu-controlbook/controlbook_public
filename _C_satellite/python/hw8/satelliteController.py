import satelliteParamHW8 as P
# PDControl import PDControl

class satelliteController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        # Instantiates the SS_ctrl object
        self.kp_phi = P.kp_phi
        self.kd_phi = P.kd_phi
        self.kp_th = P.kp_th
        self.kd_th = P.kd_th

    def update(self, phi_r, state):
        # y_r is the referenced input
        # y is the current state
        theta = state.item(0)
        phi = state.item(1)
        thetadot = state.item(2)
        phidot = state.item(3)
        # the reference angle for theta comes from the outer loop PD control
        theta_r = self.kp_phi * (phi_r - phi) - self.kd_phi * phidot
        # the torque applied to the base comes from the inner loop PD control
        tau = self.kp_th * (theta_r - theta) - self.kd_th * thetadot
        return tau







