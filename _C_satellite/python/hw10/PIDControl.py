import numpy as np

class PIDControl:
    def __init__(self, kp, ki, kd, limit, beta, Ts):
        self.kp = kp                 # Proportional control gain
        self.ki = ki                 # Integral control gain
        self.kd = kd                 # Derivative control gain
        self.limit = limit           # The output will saturate at this limit
        self.beta = beta             # gain for dirty derivative
        self.Ts = Ts                 # sample rate

        self.y_dot = 0.0              # estimated derivative of y
        self.y_d1 = 0.0              # Signal y delayed by one sample
        self.error_dot = 0.0          # estimated derivative of error
        self.error_d1 = 0.0          # Error delayed by one sample
        self.integrator = 0.0        # integrator

    def PID(self, y_r, y, flag=True):
        '''
            PID control,

            if flag==True, then returns
                u = kp*error + ki*integral(error) + kd*error_dot.
            else returns 
                u = kp*error + ki*integral(error) - kd*y_dot.

            error_dot and y_dot are computed numerically using a dirty derivative
            integral(error) is computed numerically using trapezoidal approximation
        '''

        # Compute the current error
        error = y_r - y
        # integral needs to go before derivative to update error_d1 correctly
        self.integrateError(error)
        # differentiate error and y
        self.differentiateError(error)
        self.differentiateY(y)

        # PID Control
        if flag is True:
            u_unsat = self.kp*error + self.ki*self.integrator + self.kd*self.error_dot
        else:
            u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.y_dot
        # return saturated control signal
        u_sat = self.saturate(u_unsat)
        self.integratorAntiWindup(u_sat, u_unsat)
        return u_sat

    def PD(self, y_r, y, flag=True):
        '''
            PD control,
            
            if flag==True, then returns
                u = kp*error + kd*error_dot.
            else returns 
                u = kp*error - kd*y_dot.
            
            error_dot and y_dot are computed numerically using a dirty derivative
        '''

        # Compute the current error
        error = y_r - y
        # differentiate error and y
        self.differentiateError(error)
        self.differentiateY(y)

        # PD Control
        if flag is True:
            u_unsat = self.kp*error + self.kd*self.error_dot
        else:
            u_unsat = self.kp*error - self.kd*self.y_dot
        # return saturated control signal
        u_sat = self.saturate(u_unsat)
        return u_sat

    def differentiateError(self, error):
        '''
            differentiate the error signal
        '''
        self.error_dot = self.beta*self.error_dot + (1-self.beta)*((error - self.error_d1) / self.Ts)
        self.error_d1 = error

    def differentiateY(self, y):
        '''
            differentiate y
        '''
        self.y_dot = self.beta*self.y_dot + (1-self.beta)*((y - self.y_d1) / self.Ts)
        self.y_d1 = y

    def integrateError(self, error):
        '''
            integrate error
        '''
        self.integrator = self.integrator + (self.Ts/2)*(error+self.error_d1)

    def integratorAntiWindup(self, u_sat, u_unsat):
         # integrator anti - windup
         if self.ki != 0.0:
            self.integrator = self.integrator + self.Ts/self.ki*(u_sat-u_unsat);

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







