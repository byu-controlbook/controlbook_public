import numpy as np


class pathPlanner:
    def __init__(self):
        self.z_amp = 3.0
        self.z_freq = 0.3
        self.z_offset = 6.0
        self.h_amp = 3.0
        self.h_freq = 0.2
        self.h_offset = 5.0
        self.path = np.array([[0.],  # z_path
                              [0.],  # h_path
                              [0.],  # theta
                              [0.],  # zdot_path
                              [0.],  # hdot_path
                              [0.],  # theta_dot
                              [0.0], # f_ell path
                              [0.0], # f_r path
                              ])
        
    def update(self, t):
        self.path[0][0] = self.z_amp * np.sin(self.z_freq * t) + self.z_offset
            # commanded position z
        self.path[1][0] = self.h_amp * np.sin(self.h_freq * t) + self.h_offset
            # commanded altitude h
        self.path[2][0] = 0.0
            # commanded pitch angle theta
        self.path[3][0] = self.z_amp * self.z_freq * np.cos(self.z_freq * t)
            # commanded zdot
        self.path[4][0] = self.h_amp * self.h_freq * np.cos(self.h_freq * t)
            # commanded hdot
        self.path[5][0] = 0.0
            # commanded thetadot
        self.path[6][0] = 0.0
            # commanded f_ell
        self.path[7][0] = 0.0
            # commanded f_r
        return self.path

    def z_ref(self):
        return self.path[0][0]
    
    def h_ref(self):
        return self.path[1][0]

