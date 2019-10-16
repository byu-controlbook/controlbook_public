import satelliteParamHW10 as P
from PIDControl import PIDControl

class satelliteController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        # Instantiates the SS_ctrl object
        self.phiCtrl = PIDControl(P.kp_phi, P.ki_phi, P.kd_phi, P.theta_max, P.beta, P.Ts)
        self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th, P.tau_max, P.beta, P.Ts)

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        phi_r = y_r[0]
        theta = y[0]
        phi = y[1]
        # the reference angle for theta comes from the outer loop PD control
        theta_r = self.phiCtrl.PID(phi_r, phi, flag=False)
        # the torque applied to the base comes from the inner loop PD control
        tau = self.thetaCtrl.PID(theta_r, theta, flag=False)
        return [tau]







