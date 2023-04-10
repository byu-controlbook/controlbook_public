import numpy as np
import control as cnt
import hummingbirdParam as P


class ctrlStateFeedbackIntegrator:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        wn_th =        
        zeta_th =   
        pi_th =   
        wn_psi =         
        zeta_psi = 
        wn_phi =         
        zeta_phi =   
        pi_psi =   
        # hard code Ackerman's formula
        alpha1_lon = 
        alpha2_lon = 
        alpha3_lon = 
        self.k_th = 
        self.k_thdot = 
        self.ki_lon = 
        alpha1_lat = 
        alpha2_lat = 
        alpha3_lat = 
        alpha4_lat = 
        alpha5_lat = 
        b1 = 1/P.J1x
        a1 = P.ellT*P.Fe/(P.JT+P.J1z)
        self.k_phi = 
        self.k_psi = 
        self.k_phidot = 
        self.k_psidot = 
        self.ki_lat = 
        # print gains to terminal
        print('K_lon: [', self.k_th, ',', self.k_thdot, ']')
        print('ki_lon: ', self.ki_lon)         
        print('K_lat: [', self.k_phi, ',', self.k_psi, ',', self.k_phidot, ',', self.k_psidot, ']')
        print('ki_lat: ', self.ki_lat)        
        #--------------------------------------------------
        # saturation limits
        theta_max = 30.0 * np.pi / 180.0  # Max theta, rads
        #--------------------------------------------------
        self.Ts = P.Ts
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - self.Ts) / (2 * sigma + self.Ts)
        self.phi_d1 = 0.
        self.phi_dot = 0.
        self.theta_d1 = 0.
        self.theta_dot = 0.
        self.psi_d1 = 0.
        self.psi_dot = 0.        
        # variables to implement integrator
        self.integrator_th = 0.0  
        self.error_th_d1 = 0.0  
        self.integrator_psi = 0.0  
        self.error_psi_d1 = 0.0 

    def update(self, r, y):
        theta_ref = r[0][0]
        psi_ref = r[1][0]
        phi = y[0][0]
        theta = y[1][0]
        psi = y[2][0]
        force_equilibrium =     
        # update differentiators
        self.phi_dot = 
        self.phi_d1 = 
        self.theta_dot = 
        self.theta_d1 = 
        self.psi_dot =   
        self.psi_d1 = 
        # integrate error
        error_th = theta_ref - theta
        error_psi = psi_ref - psi
        self.integrator_th = 
        self.integrator_psi = 
        self.error_th_d1 = error_th
        self.error_psi_d1 = error_psi

        # longitudinal control
        force_unsat = 
        force = saturate(force_unsat, -P.force_max, P.force_max)
        # lateral control
        torque_unsat = 
        torque = saturate(torque_unsat, -P.torque_max, P.torque_max)
        # convert force and torque to pwm signals
        pwm = np.array([[force + torque / P.d],               # u_left
                      [force - torque / P.d]]) / (2 * P.km)   # r_right          
        pwm = saturate(pwm, 0, 1)
        return pwm, np.array([[0], [theta_ref], [psi_ref]])


def saturate(u, low_limit, up_limit):
    if isinstance(u, float) is True:
        if u > up_limit:
            u = up_limit
        if u < low_limit:
            u = low_limit
    else:
        for i in range(0, u.shape[0]):
            if u[i][0] > up_limit:
                u[i][0] = up_limit
            if u[i][0] < low_limit:
                u[i][0] = low_limit
    return u
