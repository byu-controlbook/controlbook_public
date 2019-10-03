classdef satelliteDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        Js
        Jp
        k
        b
        Ts
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = satelliteDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.theta0;...      % initial base angle
                        P.phi0;...        % initial panel angle
                        P.thetadot0;...   % initial angular velocity of base
                        P.phidot0;...     % initial angular velocity of panel
                        ];     
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The parameters for any physical system are never known exactly.  Feedback
            % systems need to be designed to be robust to this uncertainty.  In the simulation
            % we model uncertainty by changing the physical parameters by a uniform random variable
            % that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
            % may change by up to 20%.  A different parameter value is chosen every time the simulation
            % is run.
            alpha = 0.2;  % Uncertainty parameter
            self.Js = P.Js * (1+2*alpha*rand-alpha);  % inertia of base
            self.Jp = P.Jp * (1+2*alpha*rand-alpha);  % inertia of panel
            self.k = P.k * (1+2*alpha*rand-alpha);    % spring coefficient
            self.b = P.b * (1+2*alpha*rand-alpha);    % Damping coefficient, Ns
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
            phi = state(2);
            thetadot = state(3);
            phidot = state(4);
            tau = u;
            % The equations of motion.
            M = [...
                self.Js, 0; 0, self.Jp;...
                ];
            c = [...
                tau - self.b*(thetadot-phidot)-self.k*(theta-phi);...
                -self.b*(phidot-thetadot)-self.k*(phi-theta);...
                ];
            tmp = M\c;
            thetaddot = tmp(1);
            phiddot = tmp(2);
            % build xdot and return
            xdot = [thetadot; phidot; thetaddot; phiddot];
        end
        %----------------------------
        function y = outputs(self)
            %
            % Returns the measured outputs as a list
            % [theta, phi] with added Gaussian noise
            % 
            % re-label states for readability
            theta = self.state(1);
            phi = self.state(2);
            % add Gaussian noise to outputs
            theta_m = theta + 0.001*randn;
            phi_m = phi + 0.001*randn;
            % return measured outputs
            y = [theta_m; phi_m];
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


