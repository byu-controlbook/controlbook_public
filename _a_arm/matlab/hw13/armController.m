classdef armController < handle
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
        Ts
    end
    methods
        %------constructor----------------------
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
        
        function [tau, x_hat] = update(self, theta_r, y)           
            % update the observer and extract z_hat
            x_hat = self.updateObserver(y);
            theta_hat = x_hat(1);

            % integrate error
            error = theta_r - theta_hat;
            self.integrateError(error);

            % compute feedback linearizing torque tau_fl
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta_hat);

            % Compute the state feedback controller
            tau_tilde = -self.K * x_hat...
                - self.ki * self.integrator;

            % compute total torque
            tau = self.saturate( tau_fl + tau_tilde);
            self.tau_d1 = tau;              
        end
        
        function xhat = updateObserver(self, y)
            % update observer using RK4 integration
            F1 = self.observer_f(self.x_hat, y);
            F2 = self.observer_f(self.x_hat + self.Ts/2*F1, y);
            F3 = self.observer_f(self.x_hat + self.Ts/2*F2, y);
            F4 = self.observer_f(self.x_hat + self.Ts*F3, y);
            self.x_hat = self.x_hat...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            xhat = self.x_hat;
        end
        
        function x_hat_dot = observer_f(self, x_hat, y)
             % compute feedback linearizing torque tau_fl
            theta_hat = x_hat(1);
            x_e = [0; 0];
            y_e = 0;
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta_hat);
            x_hat_dot = self.A * (x_hat-x_e)...
                        + self.B*(self.tau_d1 - tau_fl)...
                        + self.L*((y-y_e) - self.C * (x_hat-x_e));
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