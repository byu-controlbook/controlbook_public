classdef armController < handle
    %----------------------------
    properties
        m
        ell
        g
        theta_dot
        theta_d1
        integrator
        error_d1
        K
        ki
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %----------------------------
        function self = armController(P)
            % plant parameters known to controller
            self.m = P.m;
            self.ell = P.ell;
            self.g = P.g;
            % initialized object properties
            self.theta_dot = 0.0;
            self.theta_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.tau_max;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function tau = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            theta_r = y_r;
            theta = y(1);
            
            % compute equilibrium torque tau_e
            tau_e = self.m*self.g*(self.ell/2)*cos(theta);
            
            % differentiate theta
            self.differentiateTheta(theta);

            % integrate error
            error = theta_r - theta;
            self.integrateError(error);

            % Construct the state
            x = [theta; self.theta_dot];
            
            % Compute the state feedback controller
            tau_tilde = -self.K*x - self.ki*self.integrator;

            % compute total torque
            tau = self.saturate( tau_e + tau_tilde);
        end
        %----------------------------
        function self = differentiateTheta(self, theta)
            self.theta_dot = ...
                self.beta*self.theta_dot...
                + (1-self.beta)*((theta-self.theta_d1) / self.Ts);
            self.theta_d1 = theta;
        end
        %----------------------------
        function self = integrateError(self, error)
            self.integrator = self.integrator + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %----------------------------
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        
    end
end