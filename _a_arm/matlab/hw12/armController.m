classdef armController < handle
    properties
        m
        ell
        g
        integrator
        error_d1
        K
        ki
        limit
        Ts
    end
    methods
        function self = armController(P)
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
            % initialized object properties
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.tau_max;
            self.Ts = P.Ts;
        end
        function tau = update(self, theta_r, x)
            theta = x(1);
            % integrate error
            error = theta_r - theta;
            self.integrateError(error);            
            % compute feedback linearizing torque
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta);
            % Compute the state feedback controller
            tau_tilde = -self.K*x - self.ki*self.integrator;
            % compute total torque
            tau = self.saturate( tau_fl + tau_tilde);
        end
        function self = integrateError(self, error)
            self.integrator = self.integrator...
                + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        
    end
end