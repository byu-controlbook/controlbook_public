classdef simpleDynamics < handle
    properties
        state
        A
        B
        C
        D
        Ts
    end
    methods
        %---constructor-------------------------
        function self = simpleDynamics(P)
            self.state = [0; 0]; 
            self.A = P.A;
            self.B = P.B;
            self.C = P.C;
            self.Ts = P.Ts; 
        end
        function y = update(self, u)
            % saturate the input
            self.rk4_step(u);
            y = self.h();
        end
        function xdot = f(self, state, u)
            % Return xdot = f(x,u)
            xdot = self.A * state + self.B * u;
        end
        function y = h(self)
            % Return y = h(x)
            y = self.C * self.state;
        end
        function self = rk4_step(self, u)
            % Integrate ODE using Runge-Kutta RK4 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2*F1, u);
            F3 = self.f(self.state + self.Ts/2*F2, u);
            F4 = self.f(self.state + self.Ts*F3, u);
            self.state = self.state...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
        end
    end
end


