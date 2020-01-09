classdef armController
    properties
        kp
        kd
        tau_max
        m
        ell
        g
    end
    methods
        function self = armController(P)
            self.kp = P.kp;
            self.kd = P.kd;
            self.tau_max = P.tau_max;
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
        end
        function tau = update(self, theta_r, state)
            theta = state(1);
            thetadot = state(2);
            % compute feedback linearizing torque tau_fl
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta);
            % compute the linearized torque using PID
            tau_tilde = self.kp * (theta_r - theta)...
                - self.kd * thetadot;
            % compute total torque
            tau = tau_fl + tau_tilde;
            % saturate the final output being sent to the dynamics.
            tau = self.saturate(tau, self.tau_max);
        end
        function out = saturate(self, u, limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end
    end
end