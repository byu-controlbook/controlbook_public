classdef satelliteController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        thetaCtrl
        phiCtrl
    end
    %----------------------------
    methods
        %----------------------------
        function self = satelliteController(P)
            % Instantiates the SS_ctrl object
            self.phiCtrl = PDControl(P.kp_phi, P.kd_phi, P.theta_max, P.beta, P.Ts);
            self.thetaCtrl = PDControl(P.kp_th, P.kd_th, P.tau_max, P.beta, P.Ts);
        end
        %----------------------------
        function tau = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            phi_r = y_r;
            phi = y(2);
            theta = y(1);
            % the reference angle for theta comes from the outer loop PD control
            theta_r = self.phiCtrl.PD(phi_r, phi, false);
            % the torque applied to the base comes from the inner loop PD control
            tau = self.thetaCtrl.PD(theta_r, theta, false);
        end
    end
end