classdef pendulumController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        x_hat
        F_d1
        integrator
        error_d1
        K
        ki
        L
        A
        B
        C
        limit
        Ts
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = pendulumController(P)
            % initialized object properties
            self.x_hat = [0.0; 0.0; 0.0; 0.0];
            self.F_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.L  = P.L;
            self.A  = P.A;
            self.B  = P.B;
            self.C  = P.C;
            self.limit = P.F_max;
            self.Ts = P.Ts;
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;

            % update the observer and extract z_hat
            self.updateObserver(y);
            z_hat = self.x_hat(1);
                        
            % integrate error
            error = z_r - z_hat;
            self.integrateError(error);
            
            % Compute the state feedback controller
            F_unsat = -self.K*self.x_hat - self.ki*self.integrator;

            F_sat = self.saturate(F_unsat);
            self.updateForce(F_sat);  % the matlab handle class requires this to be in a method
            F = F_sat;
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            N = 10;
            for i=1:N
                self.x_hat = self.x_hat + self.Ts/N*(...
                    self.A*self.x_hat...
                    + self.B*self.F_d1...
                    + self.L*(y_m-self.C*self.x_hat));
            end
        end
        %----------------------------
        function self = updateForce(self, F)
            self.F_d1 = F;
        end
        %----------------------------
        function self = integrateError(self, error)
            self.integrator = self.integrator + (self.Ts/2.0)*(error+self.error_d1);
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