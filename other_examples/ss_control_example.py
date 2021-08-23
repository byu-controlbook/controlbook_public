# assuming that self.z_C (the controller state) has been
# initialized elsewhere and that the controller ss equations
# are available in the Python class
def u = updateController(self, r, y):
    # calc error signal
    error = r - y

    # Euler integration steps per control calculation
    N = 10

    # integrate differential equation for controller
    for i in range(N):
        self.z_C = self.z_C + Ts / N * (self.A_C * self.z_C \
                                        + self.B_C * error)
    u = self.C_C * self.z_C + self.D_C * error

    return u