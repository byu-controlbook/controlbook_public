classdef satelliteController < handle
    %----------------------------
    properties
        xout_C
        xout_F
        xin_C
        Aout_F
        Bout_F
        Cout_F
        Dout_F
        Aout_C
        Bout_C
        Cout_C
        Dout_C
        Ain_C
        Bin_C
        Cin_C
        Din_C
        theta_dot
        phi_dot
        theta_d1
        phi_d1
        kd_phi
        kd_th
        limit
        beta
        Ts
        N
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = satelliteController(P)
            self.xout_C = zeros(size(P.Aout_C,1),1);
            self.xout_F = zeros(size(P.Aout_F,1),1); 
            self.xin_C  = zeros(size(P.Ain_C,1),1);
            self.Aout_F = P.Aout_F;
            self.Bout_F = P.Bout_F;
            self.Cout_F = P.Cout_F;
            self.Dout_F = P.Dout_F;
            self.Aout_C = P.Aout_C;
            self.Bout_C = P.Bout_C;
            self.Cout_C = P.Cout_C;
            self.Dout_C = P.Dout_C;
            self.Ain_C = P.Ain_C;
            self.Bin_C = P.Bin_C;
            self.Cin_C = P.Cin_C;
            self.Din_C = P.Din_C;
            self.theta_dot = 0.0;
            self.phi_dot = 0.0;
            self.theta_d1 = 0.0;
            self.phi_d1 = 0.0;
            self.kd_phi = P.kd_phi;
            self.kd_th = P.kd_th;
            self.beta = P.beta;
            self.limit = P.tau_max;
            self.Ts = P.Ts;
            self.N = 10; % number of Euler integration steps for each sample
        end
        %----------------------------
        function tau = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            phi_r = y_r;
            theta = y(1);
            phi = y(2);
            
            % differentiate phi and theta
            self.differentiateTheta(theta);
            self.differentiatePhi(phi);
            
            % solve differential equation defining prefilter
            self.updatePrefilterState(phi_r);
            phi_r_filtered = self.Cout_F*self.xout_F + self.Dout_F*phi_r;
            
             % error signal for outer loop
            error_out = phi_r_filtered - phi;
                
            % Outer loop control C_out
            self.updateControlOutState(error_out);
            theta_r = -self.kd_phi*self.phi_dot + self.Cout_C*self.xout_C + self.Dout_C*error_out;

            % error signal for inner loop
            error_in = theta_r - theta;
                
            % Inner loop control C_in
            self.updateControlInState(error_in);
            tau_unsat = -self.kd_th*self.theta_dot + self.Cin_C*self.xin_C + self.Din_C*error_in;

            tau = self.saturate(tau_unsat);            
        end
        %----------------------------
        function self = updatePrefilterState(self, phi_r)
            for i=1:self.N
                self.xout_F = self.xout_F + self.Ts/self.N*(... 
                    self.Aout_F*self.xout_F + self.Bout_F*phi_r...
                    );
            end
        end
        %----------------------------
        function self = updateControlOutState(self, error_out)
            for i=1:self.N
                self.xout_C = self.xout_C + self.Ts/self.N*(...
                    self.Aout_C*self.xout_C + self.Bout_C*error_out...
                    );
            end
        end
        %----------------------------
        function self = updateControlInState(self, error_in)
            for i=1:self.N
                self.xin_C = self.xin_C + self.Ts/self.N*(...
                    self.Ain_C*self.xin_C + self.Bin_C*error_in...
                    );
            end
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