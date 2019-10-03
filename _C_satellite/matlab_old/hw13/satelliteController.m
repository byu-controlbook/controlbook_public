classdef satelliteController < handle
    %----------------------------
    properties
        x_hat
        tau_d1
        integrator
        error_d1
        K
        ki
        L
        A
        B
        C
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = satelliteController(P)
            % initialized object properties
            self.x_hat = [0.0; 0.0; 0.0; 0.0];
            self.tau_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.L  = P.L;
            self.A  = P.A;
            self.B  = P.B;
            self.C  = P.C;
            self.limit = P.tau_max;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function tau = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            phi_r = y_r;

            % update the observer and extract phi_hat
            self.updateObserver(y);
            phi_hat = self.x_hat(2);

            % integrate error
            error = phi_r - phi_hat;
            self.integrateError(error);
           
            % Compute the state feedback controller
            tau_unsat = -self.K*self.x_hat -self.ki*self.integrator;

            tau_sat = self.saturate(tau_unsat);
            self.updateTorque(tau_sat);  % the matlab handle class requires this to be in a method            
            tau = tau_sat;
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            N = 10;
            for i=1:N
                self.x_hat = self.x_hat + self.Ts/N*(...
                    self.A*self.x_hat...
                    + self.B*self.tau_d1...
                    + self.L*(y_m-self.C*self.x_hat));
            end
        end
        %----------------------------
        function self = updateTorque(self, tau)
            self.tau_d1 = tau;
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