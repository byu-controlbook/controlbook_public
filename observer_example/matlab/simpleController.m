classdef simpleController < handle
    properties
        observer_state
        u_d1
        integrator
        error_d1
        control_mode
        K
        ki
        kr
        L
        A
        B
        C
        Ts
    end
    methods
        %--------constructor--------------------
        function self = simpleController(P)
            self.u_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.control_mode = P.control_mode;
            switch self.control_mode
                case 1  % no integrator, no disturbance estimate 
                    self.observer_state = [0.0; 0.0];
                    self.K = P.K;
                    self.kr = P.kr;
                    self.A = P.A;
                    self.B = P.B;
                    self.C = P.C;
                    self.L = P.L;
                case 2 % integrator, no disturbance estimate
                    self.observer_state = [0.0; 0.0];
                    self.K = P.K;
                    self.ki = P.ki;
                    self.A = P.A;
                    self.B = P.B;
                    self.C = P.C;
                    self.L = P.L;        
                case 3 % no integrator, disturbance estimate
                    self.observer_state = [0.0; 0.0; 0.0];
                    self.K = P.K;
                    self.kr = P.kr;
                    self.A = P.A2;
                    self.B = P.B1;
                    self.C = P.C2;
                    self.L = P.L2;
                case 4 % integrator, disturbance estimate
                    self.observer_state = [0.0; 0.0; 0.0];
                    self.K = P.K;
                    self.ki = P.ki;
                    self.A = P.A2;
                    self.B = P.B1;
                    self.C = P.C2;
                    self.L = P.L2;
            end
            self.Ts = P.Ts;
        end
        function [u, x_hat, d_hat] = update(self, r, y)
            % update the observer and extract z_hat
            [x_hat, d_hat] = self.updateObserver(y);

            % integrate error
            error = r - y;
            self.integrateError(error);

            % controller
            switch self.control_mode
                case 1  % no integrator, no disturbance estimate 
                    u = -self.K * x_hat...
                        + self.kr * r;
                case 2 % integrator, no disturbance estimate
                    u = -self.K * x_hat...
                        -self.ki * self.integrator;
                case 3 % no integrator, disturbance estimate
                    u = -self.K * x_hat...
                        +self.kr * r...
                        -d_hat;
                case 4 % integrator, disturbance estimate
                    u = -self.K * x_hat...
                        -self.ki * self.integrator...
                        -d_hat;
            end
            self.u_d1 = u;
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
            switch self.control_mode
                case 1  % no integrator, no disturbance estimate 
                    dhat = 0;
                case 2 % integrator, no disturbance estimate
                    dhat = 0;
                case 3 % no integrator, disturbance estimate
                    dhat = self.observer_state(3);
                case 4 % integrator, disturbance estimate
                    dhat = self.observer_state(3);
            end
        end
        function x_hat_dot = observer_f(self, observer_state, y)
            x_hat_dot = self.A * observer_state...
                        + self.B * self.u_d1...
                        + self.L * (y - self.C * observer_state);
        end
        function self = integrateError(self, error)
            self.integrator = self.integrator ...
                + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
    end
end