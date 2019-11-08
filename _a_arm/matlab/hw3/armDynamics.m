classdef armDynamics < handle
    properties
        state
        output
        m
        ell
        b
        g
        Ts
        torque_limit
    end
    methods
        %---constructor-------------------------
        function self = armDynamics(alpha, P)
            % Initial state conditions
            self.state = [...
                        P.theta0;...  % initial angle
                        P.thetadot0;...  % initial angular rate
                        ]; 
            % Mass of the arm, kg
            self.m = P.m * (1+2*alpha*rand-alpha);  
            % Length of the arm, m
            self.ell = P.ell * (1+2*alpha*rand-alpha);  
            % Damping coefficient, Ns
            self.b = P.b * (1+2*alpha*rand-alpha);  
            % gravity constant is well known and so don't change.
            self.g = P.g;  
            % sample rate at which dynamics is propagated
            self.Ts = P.Ts; 
            self.torque_limit = P.tau_max;
        end
        function y = update(self, u)
            % saturate the input
            u = self.saturate(u, self.torque_limit);
            self.rk4_step(u);
            y = self.h();
            self.output = y;
        end
        function xdot = f(self, state, u)
            % Return xdot = f(x,u)
            theta = state(1);
            thetadot = state(2);
            tau = u;
            % The equations of motion.
            thetaddot = (3/self.m/self.ell^2)...
                * (tau - self.b * thetadot...
                   - self.m * self.g * self.ell / 2 * cos(theta)); 
            xdot = [thetadot; thetaddot];
        end
        function y = h(self)
            % Return y = h(x)
            theta = self.state(1);
            y = theta;
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
        function out = saturate(self, in, limit)
            if abs(in) > limit
                out = limit * sign(in);
            else 
                out = in;
            end
        end
    end
end


