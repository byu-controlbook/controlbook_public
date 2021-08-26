import satelliteParamHW8 as P

class satelliteController:
    def __init__(self):
        self.kp_phi = P.kp_phi
        self.kd_phi = P.kd_phi
        self.kp_th = P.kp_th
        self.kd_th = P.kd_th

    def update(self, phi_r, state):

        theta = state.item(0)
        phi = state.item(1)
        thetadot = state.item(2)
        phidot = state.item(3)

        # outer loop: outputs the reference angle for theta
        # note that book recommends a feed forward term because
        # of poor DC gain on the outer loop which
        # is why we add an addition "phi_r" at the end
        theta_r = self.kp_phi * (phi_r - phi) \
                  - self.kd_phi * phidot + phi_r

        # inner loop: outputs the torque applied to the base
        tau = self.kp_th * (theta_r - theta) \
              - self.kd_th * thetadot

        return tau
