classdef dynamics < handle
    properties
        state
        Ts
    end
    methods
        %---constructor-------------------------
        function self = dynamics(P)
            % Initial state conditions
            self.state = 0; 
            % sample rate at which dynamics is propagated
            self.Ts = P.Ts; 
        end
        function y = update(self, u)
            self.rk4_step(u);
            y = self.h();
        end
        function xdot = f(self, state, u)
            % Return xdot = f(x,u)
            xdot = -state + u;
        end
        function y = h(self)
            % Return y = h(x)
            y = self.state;
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


