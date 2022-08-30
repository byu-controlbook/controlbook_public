import hw10.satelliteParamHW10 as P
from hw10.PIDControl import PIDControl

class satelliteController:
    def __init__(self):
        # Instantiates the SS_ctrl object
        self.phiCtrl = PIDControl(P.kp_phi, P.ki_phi, P.kd_phi,
                                  P.theta_max, P.sigma, P.Ts)
        self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th,
                                    P.tau_max, P.sigma, P.Ts)

    def update(self, phi_r, y):
        theta = y[0,0]
        phi = y[1,0]

        # the reference angle for theta comes from
        # the outer loop PD control
        theta_r = self.phiCtrl.PID(phi_r, phi, flag=False)

        # the torque applied to the base comes from
        # the inner loop PD control
        tau = self.thetaCtrl.PD(theta_r, theta, flag=False)

        return tau







