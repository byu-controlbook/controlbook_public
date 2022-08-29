classdef pendulumController < handle
    properties
        integrator
        error_d1
        K
        ki
        limit
        Ts
    end
    methods
        %--Constructor--------------------------
        function self = pendulumController(P)
            % initialized object properties
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.F_max;
            self.Ts = P.Ts;
        end
        
        function F = update(self, z_r, x)
            z = x(1);
            % integrate error
            error = z_r - z;
            self.integrateError(error);
            % Compute the state feedback controller
            F_unsat = -self.K*x - self.ki*self.integrator;
            F_sat = self.saturate(F_unsat);
            F = F_sat;
        end
        
        function self = integrateError(self, error)
            self.integrator = self.integrator...
                + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %----------------------------
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
    end
end