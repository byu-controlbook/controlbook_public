classdef pendulumController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        thetaCtrl
    end
    %----------------------------
    methods
        %----------------------------
        function self = pendulumController(P)
            % Instantiates the SS_ctrl object
            self.zCtrl = PIDControl(P.kp_z, P.ki_z, P.kd_z, P.theta_max, P.beta, P.Ts);
            self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th, P.F_max, P.beta, P.Ts);
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            theta = y(2);
            % the reference angle for theta comes from the outer loop PID control
            theta_r = self.zCtrl.PID(z_r, z, false);
            % the force applied to the cart comes from the inner loop PD control
            F = self.thetaCtrl.PD(theta_r, theta, false);
        end
    end
end