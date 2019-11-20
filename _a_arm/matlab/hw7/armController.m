classdef armController
    properties
        kp
        kd
        m
        ell
        g
    end
    methods
        %----------------------------
        function self = armController(P)
            self.kp = P.kp;
            self.kd = P.kd;
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
        end
        function tau = update(self, theta_r, state)
            theta = state(1);
            thetadot = state(2);
            % compute the feedback linarized torque tau_fl
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta);
            % compute the linearized torque using PID
            tau_tilde = self.kp * (theta_r - theta)...
                - self.kd * thetadot;
            % compute total torque
            tau = tau_fl + tau_tilde;
        end
    end
end