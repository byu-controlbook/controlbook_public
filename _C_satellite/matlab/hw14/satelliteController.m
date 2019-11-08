classdef satelliteController < handle
    properties
        observer_state
        d_hat       
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
        %--Constructor--------------------------
        function self = satelliteController(P)
            % initialized object properties
            self.observer_state = [0.0; 0.0; 0.0; 0.0; 0.0];
            self.tau_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.L  = P.L2;
            self.A  = P.A2;
            self.B  = P.B1;
            self.C  = P.C2;
            self.limit = P.tau_max;
            self.Ts = P.Ts;
        end
        %----------------------------
        function [tau, x_hat, d_hat] = update(self, phi_r, y)
            % update the observer and extract phi_hat
            [x_hat, d_hat] = self.updateObserver(y);
            phi_hat = x_hat(2);

            % integrate error
            error = phi_r - phi_hat;
            self.integrateError(error);
           
            % Compute the state feedback controller
            tau_unsat = -self.K * x_hat...
                - self.ki * self.integrator...
                - d_hat;
            tau_sat = self.saturate(tau_unsat);
            self.tau_d1 = tau_sat;
            tau = tau_sat;
        end
        function [x_hat, d_hat] = updateObserver(self, y)
            % update observer using RK4 integration
            F1 = self.observer_f(self.observer_state, y);
            F2 = self.observer_f(self.observer_state + self.Ts/2*F1, y);
            F3 = self.observer_f(self.observer_state + self.Ts/2*F2, y);
            F4 = self.observer_f(self.observer_state + self.Ts*F3, y);
            self.observer_state = self.observer_state...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            x_hat = self.observer_state(1:4);
            d_hat = self.observer_state(5);
        end
        function x_hat_dot = observer_f(self, observer_state, y)
            x_hat_dot = self.A * observer_state...
                    + self.B * self.tau_d1...
                    + self.L * (y - self.C * observer_state);
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