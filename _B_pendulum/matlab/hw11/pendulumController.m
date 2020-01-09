classdef pendulumController < handle
    properties
        K
        kr
        limit
        Ts
    end
    methods
        %--Constructor--------------------------
        function self = pendulumController(P)
            % initialized object properties
            self.K = P.K;
            self.kr = P.kr;
            self.limit = P.F_max;
            self.Ts = P.Ts;
        end
        
        function F = update(self, z_r, x)
            z = x(1);
            % Compute the state feedback controller
            F_unsat = -self.K*x + self.kr*z_r;
            F_sat = self.saturate(F_unsat);
            F = F_sat;
        end
        
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
    end
end