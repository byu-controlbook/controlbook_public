import numpy as np
import pendulumParam as P
import hw8.pendulumParamHW8 as P8

class pendulumController:
    def __init__(self):
        self.kp_z = P8.kp_z
        self.kd_z = P8.kd_z
        self.kp_th = P8.kp_th
        self.kd_th = P8.kd_th
        self.filter = zeroCancelingFilter()
        self.F_max = P8.F_max

    def update(self, z_r, state):
        z = state[0,0]
        theta = state[1,0]
        zdot = state[2,0]
        thetadot = state[3,0]

        # the reference angle for theta comes from the
        # outer loop PD control
        tmp = self.kp_z * (z_r - z) - self.kd_z * zdot

        # low pass filter the outer loop to cancel
        # left-half plane zero and DC-gain
        theta_r = self.filter.update(tmp)

        # the force applied to the cart comes from the
        # inner loop PD control
        F = self.kp_th * (theta_r - theta) - self.kd_th * thetadot

        # saturate the input based on actuator limits
        F = self.saturate(F, self.F_max)
        
        return F

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
        return u



class zeroCancelingFilter:
    def __init__(self):
        # dividing by the DC gain here allows us to cancel its effect 
        # on other parts of the controller. 
        self.a = (1/P8.DC_gain)  * (-3.0/(2.0*P.ell))
        self.b = np.sqrt(3.0*P.g/(2.0*P.ell))
        self.filt_output = 0.0

    def update(self, input):
        # integrate using RK1
        # this might work better if discretized using z-transform 
        # (see methods from Chapter 10, and frequency response 
        # methods from Chapters 15-18)
        self.filt_output = self.filt_output \
                     + P.Ts * (-self.b*self.filt_output+ self.a*input)
        return self.filt_output






