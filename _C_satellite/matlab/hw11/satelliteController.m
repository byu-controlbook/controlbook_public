classdef satelliteController < handle
    %----------------------------
    properties
        init_flag
        theta_dot
        phi_dot
        theta_d1
        phi_d1
        K
        kr
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = satelliteController(P)
            % initialized object properties
            self.init_flag = 1;
            self.theta_dot = 0.0;
            self.phi_dot = 0.0;
            self.theta_d1 = 0.0;
            self.phi_d1 = 0.0;
            self.K = P.K;
            self.kr = P.kr;
            self.limit = P.tau_max;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function tau = update(self, phi_r, y)
            theta = y(1);
            phi = y(2);

            % differentiate phi and theta
            if self.init_flag==1
                self.phi_d1=phi;
                self.theta_d1=theta;
                self.init_flag=0;
            end
            self.differentiateTheta(theta);
            self.differentiatePhi(phi);

            % Construct the state
            x = [theta; phi; self.theta_dot; self.phi_dot;];
            
            % Compute the state feedback controller
            tau_unsat = -self.K*x + self.kr*phi_r;

            tau_sat = self.saturate(tau_unsat);
            tau = tau_sat;
        end
        %----------------------------
        function self = differentiatePhi(self, phi)
            self.phi_dot = ...
                self.beta*self.phi_dot...
                + (1-self.beta)*((phi - self.phi_d1) / self.Ts);
            self.phi_d1 = phi;            
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