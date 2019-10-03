classdef pendulumController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
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
        limit
        Ts
        N
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = pendulumController(P)
            % initialized object properties
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
            self.limit = P.F_max;
            self.Ts = P.Ts;
            self.N = 10; % number of Euler integration steps for each sample
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            theta = y(2);
            
            % solve differential equation defining prefilter
            self.updatePrefilterState(z_r);
            z_r_filtered = self.Cout_F*self.xout_F + self.Dout_F*z_r;
                
            % error signal for outer loop
            error_out = z_r_filtered - z;
                
            % Outer loop control C_out
            self.updateControlOutState(error_out);
            theta_r = self.Cout_C*self.xout_C + self.Dout_C*error_out;

            % error signal for inner loop
            error_in = theta_r - theta;
                
            % Inner loop control C_in
            self.updateControlInState(error_in);
            F_unsat = self.Cin_C*self.xin_C + self.Din_C*error_in;

            F = self.saturate(F_unsat);
        end
        %----------------------------
        function self = updatePrefilterState(self, z_r)
            for i=1:self.N
                self.xout_F = self.xout_F + self.Ts/self.N*(... 
                    self.Aout_F*self.xout_F + self.Bout_F*z_r...
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
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
    end
end