classdef satelliteController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        kp_phi
        kd_phi
        kp_th
        kd_th
    end
    %----------------------------
    methods
        %----------------------------
        function self = satelliteController(P)
            self.kp_phi = P.kp_phi;
            self.kd_phi = P.kd_phi;
            self.kp_th = P.kp_th;
            self.kd_th = P.kd_th;
        end
        %----------------------------
        function tau = update(self, phi_r, state)
            theta = state(1);
            phi = state(2);
            thetadot = state(3);
            phidot = state(4);
            % the reference angle for theta comes from the outer loop PD control
            theta_r = self.kp_phi * (phi_r - phi) - self.kd_phi * phidot;
            % the torque applied to the base comes from the inner loop PD control
            tau = self.kp_th * (theta_r - theta) - self.kd_th * thetadot;
        end
    end
end