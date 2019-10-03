classdef armController < handle
    %----------------------------
    properties
        m
        ell
        g
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
        %----------------------------
        function self = armController(P)
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
            % initialized object properties
            self.x_hat = [0.0; 0.0];
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
            self.Ts = P.Ts;
        end
        %----------------------------
        function tau = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            theta_r = y_r;
            
            % update the observer and extract z_hat
            self.updateObserver(y);
            theta_hat = self.x_hat(1);

            % integrate error
            error = theta_r - theta_hat;
            self.integrateError(error);

            % compute equilibrium torque tau_e at old angle
            theta_hat = self.x_hat(1);
            tau_e = self.m*self.g*(self.ell/2)*cos(theta_hat);

            % Compute the state feedback controller
            tau_tilde = -self.K*self.x_hat - self.ki*self.integrator;

            % compute total torque
            tau = self.saturate( tau_e + tau_tilde);
            self.updateTorque(tau);  % the matlab handle class requires this to be in a method
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            % compute equilibrium torque tau_e at old angle
            theta_hat = self.x_hat(1);
            tau_e = self.m*self.g*(self.ell/2)*cos(theta_hat);

            N = 10;
            for i=1:N
                self.x_hat = self.x_hat + self.Ts/N*(...
                    self.A*self.x_hat...
                    + self.B*(self.tau_d1-tau_e)...
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