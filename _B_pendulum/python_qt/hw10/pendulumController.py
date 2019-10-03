import pendulumParamHW10 as P
from PIDControl import PIDControl

class pendulumController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        # Instantiates the SS_ctrl object
        self.zCtrl = PIDControl(P.kp_z, P.ki_z, P.kd_z, P.theta_max, P.beta, P.Ts)
        self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th, P.F_max, P.beta, P.Ts)

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]
        theta = y[1]
        # the reference angle for theta comes from the outer loop PID control
        theta_r = self.zCtrl.PID(z_r, z, flag=False)
        # the force applied to the cart comes from the inner loop PD control
        F = self.thetaCtrl.PD(theta_r, theta, flag=False)
        return [F]







