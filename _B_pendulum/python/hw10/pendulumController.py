import numpy as np
import pendulumParam as P
import pendulumParamHW10 as P10
from PIDControl import PIDControl

class pendulumController:
    def __init__(self):
        # Instantiates the SS_ctrl object
        self.zCtrl = PIDControl(P10.kp_z, P10.ki_z, P10.kd_z,
                                P10.theta_max, P10.sigma, P10.Ts)
        self.thetaCtrl = PIDControl(P10.kp_th, 0.0, P10.kd_th,
                                    P10.F_max, P10.sigma, P10.Ts)
        self.filter = zeroCancelingFilter()

    def update(self, z_r, y):
        z = y.item(0)
        theta = y.item(1)
        # the reference angle for theta comes from
        # the outer loop PID control
        theta_r = self.zCtrl.PID(z_r, z, flag=False)
        # low pass filter the outer loop to cancel
        # left-half plane zero and DC-gain
        theta_r = self.filter.update(theta_r)
        # the force applied to the cart comes from
        # the inner loop PD control
        F = self.thetaCtrl.PD(theta_r, theta, flag=False)
        return F

class zeroCancelingFilter:
    def __init__(self):
        self.a = -3.0/(2.0*P.ell*P10.DC_gain)
        self.b = np.sqrt(3.0*P.g/(2.0*P.ell))
        self.state = 0.0

    def update(self, input):
        # integrate using RK1
        self.state = self.state \
                     + P.Ts * (-self.b*self.state + self.a*input)
        return self.state






