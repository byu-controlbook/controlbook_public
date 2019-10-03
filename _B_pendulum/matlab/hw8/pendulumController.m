classdef pendulumController  < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        kp_z
        kd_z
        kp_th
        kd_th
    end
    %----------------------------
    methods
        %----------------------------
        function self = pendulumController(P)
            % Instantiates the SS_ctrl object
            self.kp_z = P.kp_z;
            self.kd_z = P.kd_z;
            self.kp_th = P.kp_th;
            self.kd_th = P.kd_th;
        end
        %----------------------------
        function F = update(self, z_r, state)
            z = state(1);
            theta = state(2);
            zdot = state(3);
            thetadot = state(4);
            % the reference angle for theta comes from the outer loop PD control
            theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot;
            % the force applied to the cart comes from the inner loop PD control
            F = self.kp_th * (theta_r - theta) - self.kd_th * thetadot;
        end
    end
end