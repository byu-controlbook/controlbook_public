classdef armController < handle
    properties
        m
        ell
        g
        control
        prefilter
        limit
    end
    methods
        function self = armController(P)
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
            % initialized control and prefilter
            self.prefilter = transferFunction(P.num_F, P.den_F, P.Ts);
            self.control = transferFunction(P.num_C, P.den_C, P.Ts);
            self.limit = P.tau_max;
        end
        function tau = update(self, theta_r, y)
            theta = y(1);
            
            % prefilter the reference
            theta_r_filtered = self.prefilter.update(theta_r);

            % define error and update the controller
            error = theta_r_filtered - theta;
            tau_tilde = self.control.update(error);

            % compute feedback linearlized torque tau_fl
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta);

            % compute total torque
            tau = self.saturate( tau_fl + tau_tilde);
        end
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        
    end
end