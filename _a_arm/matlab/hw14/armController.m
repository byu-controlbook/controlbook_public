classdef armController < handle
    properties
        m
        ell
        g
        observer_state
        tau_d1
        integrator
        error_d1
        K
        ki
        L
        Ld
        A
        B
        C
        limit
        Ts
    end
    methods
        %--------constructor--------------------
        function self = armController(P)
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
            % initialized object properties
            self.observer_state = [0.0; 0.0; 0.0];
            self.tau_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.A = P.A2;
            self.B  = P.B1;
            self.C  = P.C2;
            self.L  = P.L2;
            self.limit = P.tau_max;
            self.Ts = P.Ts;
        end
        function [tau, x_hat, d_hat] = update(self, theta_r, y)
            % update the observer and extract z_hat
            [x_hat, d_hat] = self.updateObserver(y);
            theta_hat = x_hat(1);

            % integrate error
            error = theta_r - theta_hat;
            self.integrateError(error);

            % compute feedback linearizing torque tau_fl
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta_hat);

            % Compute the state feedback controller
            tau_tilde = -self.K * x_hat...
                - self.ki * self.integrator...
                - d_hat;

            % compute total torque
            tau = self.saturate( tau_fl + tau_tilde);
            self.tau_d1 = tau;  
        end
        function [xhat, dhat] = updateObserver(self, y)
            % update observer using RK4 integration
            F1 = self.observer_f(self.observer_state, y);
            F2 = self.observer_f(self.observer_state + self.Ts/2*F1, y);
            F3 = self.observer_f(self.observer_state + self.Ts/2*F2, y);
            F4 = self.observer_f(self.observer_state + self.Ts*F3, y);
            self.observer_state = self.observer_state ...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            xhat = self.observer_state(1:2);
            dhat = self.observer_state(3);
        end
        function x_hat_dot = observer_f(self, observer_state, y)
            % compute feedback linearizing torque tau_fl
            theta_hat = observer_state(1);
            tau_fl = self.m*self.g*(self.ell/2)*cos(theta_hat);
            x_hat_dot = self.A * observer_state...
                        + self.B * (self.tau_d1 - tau_fl)...
                        + self.L * (y - self.C * observer_state);
        end
        function self = integrateError(self, error)
            self.integrator = self.integrator ...
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