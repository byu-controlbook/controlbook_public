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
            self.phiCtrl = PIDControl(P.kp_phi, P.ki_phi, P.kd_phi, P.theta_max, P.beta, P.Ts);
            self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th, P.tau_max, P.beta, P.Ts);
        end
        %----------------------------
        function tau = update(self, phi_r, output)
            theta = output(1);
            phi = output(2);
            % the reference angle for theta comes from the outer loop PD control
            theta_r = self.phiCtrl.PID(phi_r, phi, false);
            % the torque applied to the base comes from the inner loop PD control
            tau = self.thetaCtrl.PD(theta_r, theta, false);
        end
    end
end