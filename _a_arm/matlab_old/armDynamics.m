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
        end
        %----------------------------
        function self = propagateDynamics(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, u);
            k2 = self.derivatives(self.state + self.Ts/2*k1, u);
            k3 = self.derivatives(self.state + self.Ts/2*k2, u);
            k4 = self.derivatives(self.state + self.Ts*k3, u);
            self.state = self.state + self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4);
        end
        %----------------------------
        function xdot = derivatives(self, state, u)
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
        function y = outputs(self)
            %
            % Returns the measured outputs as a list
            % [z, theta] with added Gaussian noise
            % 
            % re-label states for readability
            theta = self.state(1);
            % add Gaussian noise to outputs
            theta_m = theta + 0.001*randn;
            % return measured outputs
            y = [theta_m];
        end
        %----------------------------
        function x = states(self)
            %
            % Returns all current states as a list
            %
            x = self.state;
        end
    end
end


