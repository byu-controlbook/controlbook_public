classdef armDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        m
        ell
        b
        g
        Ts
        torque_limit
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = armDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.theta0;...      % initial angle
                        P.thetadot0;...   % initial angular rate
                        ]; 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The parameters for any physical system are never known exactly.  Feedback
            % systems need to be designed to be robust to this uncertainty.  In the simulation
            % we model uncertainty by changing the physical parameters by a uniform random variable
            % that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
            % may change by up to 20%.  A different parameter value is chosen every time the simulation
            % is run.
            alpha = 0.2;  % Uncertainty parameter
            self.m = P.m * (1+2*alpha*rand-alpha);  % Mass of the arm, kg
            self.ell = P.ell * (1+2*alpha*rand-alpha);  % Length of the arm, m
            self.b = P.b * (1+2*alpha*rand-alpha);  % Damping coefficient, Ns
            self.g = P.g;  % the gravity constant is well known and so we don't change it.
            self.Ts = P.Ts; % sample rate at which dynamics is propagated
            self.torque_limit = P.tau_max;
        end
        %----------------------------
        function y = update(self, u)
            % saturate the input
            u = self.saturate(u, self.torque_limit);
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
            theta = state(1);
            thetadot = state(2);
            tau = u;
            % The equations of motion.
            thetaddot = (3/self.m/self.ell^2)*(tau - self.b*thetadot - self.m*self.g*self.ell/2*cos(theta)); 

            % build xdot and return
            xdot = [thetadot; thetaddot];
        end
        %----------------------------
        function y = h(self)
            %
            % Returns the measured outputs as a list
            % [z, theta] with added Gaussian noise
            % 
            % re-label states for readability
            theta = self.state(1);
            % add Gaussian noise to outputs
            %theta_m = theta + 0.001*randn;
            % return measured outputs
            y = [theta];
        end
        %----------------------------
        function out = saturate(self, in, limit)
            if abs(in) > limit
                out = limit * sign(in);
            else 
                out = in;
            end
        end
    end
end


