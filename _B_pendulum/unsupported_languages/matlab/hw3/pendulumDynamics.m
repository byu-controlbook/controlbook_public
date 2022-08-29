classdef pendulumDynamics < handle
    properties
        state
        output
        m1
        m2
        ell
        b
        g
        Ts
        force_limit
    end
    methods
        %---constructor-------------------------
        function self = pendulumDynamics(alpha, P)
            % Initial state conditions
            self.state = [...
                        P.z0;...          % z initial position
                        P.theta0;...      % Theta initial orientation
                        P.zdot0;...       % zdot initial velocity
                        P.thetadot0;...   % Thetadot initial velocity
                        ];  
            self.m1 = P.m1 * (1+2*alpha*rand-alpha);  % Mass of the pendulum, kg
            self.m2 = P.m2 * (1+2*alpha*rand-alpha);  % Mass of the cart, kg
            self.ell = P.ell * (1+2*alpha*rand-alpha);  % Length of the rod, m
            self.b = P.b * (1+2*alpha*rand-alpha);  % Damping coefficient, Ns
            self.g = P.g;  % the gravity constant is well known and so we don't change it.
            self.Ts = P.Ts; % sample rate at which dynamics is propagated
            self.force_limit = P.F_max;
        end
        %----------------------------
        function y = update(self, u)
            % saturate the input
            u = self.saturate(u, self.force_limit);
            self.rk4_step(u);
            y = self.h();
            self.output = y;
        end
        function xdot = f(self, state, u)
            % Return xdot = f(x,u), 
            z = state(1);
            theta = state(2);
            zdot = state(3);
            thetadot = state(4);
            F = u;
            % The equations of motion.
            M = [self.m1+self.m2, self.m1*(self.ell/2.0)*cos(theta);...
                 self.m1*(self.ell/2.0)*cos(theta), self.m1*(self.ell^2/3.0)];
            C = [self.m1*(self.ell/2.0)*thetadot^2*sin(theta) + F - self.b*zdot;...
                 self.m1*self.g*(self.ell/2.0)*sin(theta)];
            tmp = M\C;
            zddot = tmp(1);
            thetaddot = tmp(2);
            xdot = [zdot; thetadot; zddot; thetaddot];
        end
        function y = h(self)
            % return y = h(x)
            z = self.state(1);
            theta = self.state(2);
            y = [z; theta];
        end
        function self = rk4_step(self, u)
            % Integrate ODE using Runge-Kutta RK4 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2*F1, u);
            F3 = self.f(self.state + self.Ts/2*F2, u);
            F4 = self.f(self.state + self.Ts*F3, u);
            self.state = self.state + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
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


