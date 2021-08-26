# assuming that self.z_C and self.z_F have been initialized
# elsewhere and that the controller AND prefilter ss equations
# (e.g. A_C, B_C, C_C, D_C, A_F, B_F, etc.) are available from
# the Python class

def u = updateController(self, r, y):

    # Euler integration steps per filter calculation
    N = 10

    # integrate differential equation for controller
    for i in range(N):
        # prefilter the reference command
        self.z_F = self.z_F + Ts / N * (self.A_F * self.z_F \
                                        + self.B_F * r)
        r_filtered = self.C_F * self.z_F + self.D_F * r

        # error signal uses filtered reference
        error = r_filtered - y
        self.z_C = self.z_C + Ts / N * (self.A_C * self.z_C \
                                        + self.B_C * error)

    u = self.C_C * self.z_C + self.D_C * error

    return u