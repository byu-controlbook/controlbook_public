classdef armController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        %thetaCtrl
        kp
        kd
        m
        ell
        g
    end
    %----------------------------
    methods
        %----------------------------
        function self = armController(P)
            self.kp = P.kp;
            self.kd = P.kd;
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
        end
        %----------------------------
        function tau = update(self, theta_r, state)
            theta = state(1);
            thetadot = state(2);
            
            % compute the feedback linarized torque tau_fl
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta);
            % compute equilibrium torque tau_e
            theta_e = 0;
            tau_e = self.m*self.g*(self.ell/2)*cos(theta_e);
            % compute the linearized torque using PID
            tau_tilde = self.kp * (theta_r - theta) - self.kd * thetadot;
            % compute total torque
            %tau = tau_e + tau_tilde;
            tau = tau_fl + tau_tilde;
        end
    end
end