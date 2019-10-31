classdef pendulumController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        init_flag
        z_dot
        theta_dot
        z_d1
        theta_d1
        K
        kr
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = pendulumController(P)
            % initialized object properties
            self.init_flag = 1;
            self.z_dot = 0.0;
            self.theta_dot = 0.0;
            self.z_d1 = 0.0;
            self.theta_d1 = 0.0;
            self.K = P.K;
            self.kr = P.kr;
            self.limit = P.F_max;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function F = update(self, z_r, y)
            z = y(1);
            theta = y(2);

            % differentiate z and theta
            if self.init_flag==1
                self.z_d1=z;
                self.theta_d1=theta;
                self.init_flag=0;
            end
            self.differentiateZ(z);
            self.differentiateTheta(theta);

            % Construct the state
            x = [z; theta; self.z_dot; self.theta_dot];
            
            % Compute the state feedback controller
            F_unsat = -self.K*x + self.kr*z_r;

            F_sat = self.saturate(F_unsat);
            F = F_sat;
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z - self.z_d1) / self.Ts);
            self.z_d1 = z;            
        end
        %----------------------------
        function self = differentiateTheta(self, theta)
            self.theta_dot = ...
                self.beta*self.theta_dot...
                + (1-self.beta)*((theta-self.theta_d1) / self.Ts);
            self.theta_d1 = theta;
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