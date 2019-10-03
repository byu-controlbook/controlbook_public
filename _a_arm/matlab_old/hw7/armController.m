classdef armController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        thetaCtrl
        m
        ell
        g
    end
    %----------------------------
    methods
        %----------------------------
        function self = armController(P)
            % Instantiates the SS_ctrl object
            self.thetaCtrl = PDControl(P.kp, P.kd, P.tau_max, P.beta, P.Ts);
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
        end
        %----------------------------
        function tau = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            theta_r = y_r;
            theta = y(1);
            
            % compute equilibrium torque tau_e
            tau_e = self.m*self.g*(self.ell/2)*cos(theta);
            % compute the linearized torque using PID
            tau_tilde = self.thetaCtrl.PD(theta_r, theta, false);
            % compute total torque
            tau = tau_e + tau_tilde;
        end
    end
end