classdef armController
    properties
        thetaCtrl
        m
        ell
        g
    end
    methods
        %----------------------------
        function self = armController(P)
            % Instantiates the SS_ctrl object
            self.thetaCtrl = PIDControl(P.kp, P.ki, P.kd,...
                P.tau_max, P.sigma, P.Ts);
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
        end
        function tau = update(self, theta_r, y)
            theta = y(1);
            % compute feedback-linearizing torque tau_fl
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta);
            % compute the linearized torque using PID
            tau_tilde = self.thetaCtrl.PID(theta_r, theta, false);
            % compute total torque
            tau = tau_fl + tau_tilde;
        end
    end
end