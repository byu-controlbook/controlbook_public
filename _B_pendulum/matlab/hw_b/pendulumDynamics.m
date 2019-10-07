classdef pendulumDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        m1
        m2
        ell
        b
        g
        Ts
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = pendulumDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.z0;...          % z initial position
                        P.theta0;...      % Theta initial orientation
                        P.zdot0;...       % zdot initial velocity
                        P.thetadot0;...   % Thetadot initial velocity
                        ];     
            self.m1 = P.m1;  % Mass of the pendulum, kg
            self.m2 = P.m2;  % Mass of the cart, kg
            self.ell = P.ell;  % Length of the rod, m
            self.b = P.b;  % Damping coefficient, Ns
            self.g = P.g;  % the gravity constant is well known and so we don't change it.
            self.Ts = P.Ts; % sample rate at which dynamics is propagated
          
        end
        %----------------------------
        function y = update(self, u)
            self.rk4_step(u);
            y = self.h();
        end
        %----------------------------
        function self = rk1_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK1 algorithm
            self.state = self.state + self.Ts * self.f(self.state, u);
        end
        %----------------------------
        function self = rk2_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK2 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2 * F1, u);
            self.state = self.state + self.Ts/6 * (F1 + F2);
        end
        %----------------------------
        function self = rk4_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2*F1, u);
            F3 = self.f(self.state + self.Ts/2*F2, u);
            F4 = self.f(self.state + self.Ts*F3, u);
            self.state = self.state + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
        end
        %----------------------------
        function xdot = f(self, state, u)
            %
            % Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
            % 
            % re-label states and inputs for readability
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
            % build xdot and return
            xdot = [zdot; thetadot; zddot; thetaddot];
        end
        %----------------------------
        function y = h(self)
            %
            % Returns the measured outputs as a list
            % [z, theta] with added Gaussian noise
            % 
            % re-label states for readability
            z = self.state(1);
            theta = self.state(2);
            % return measured outputs
            y = [z; theta];
        end
    end
end


