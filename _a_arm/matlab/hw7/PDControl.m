classdef PDControl < handle
    %----------------------------
    properties
        kp
        kd
        limit
        beta
        Ts
        
        y_dot
        y_d1
        error_dot
        error_d1
    end
    %----------------------------
    methods
        %----------------------------
        function self = PDControl(kp, kd, limit, beta, Ts)
            self.kp = kp;                 % Proportional control gain
            self.kd = kd;                 % Derivative control gain
            self.limit = limit;           % The output will saturate at this limit
            self.beta = beta;
            self.Ts = Ts;

            self.y_dot = 0.0;              % estimated derivative of y
            self.y_d1 = 0.0;              % Signal y delayed by one sample
            self.error_dot = 0.0;          % estimated derivative of error
            self.error_d1 = 0.0;          % Error delayed by one sample
        end
        %----------------------------
        function u = PD(self, y_r, y, flag)
            %
            %    PD control,
            %    
            %    if flag==True, then returns
            %        u = kp*error + kd*error_dot.
            %    else returns 
            %        u = kp*error - kd*y_dot.
            %    
            %    error_dot and y_dot are computed numerically using a dirty derivative
            %

            % Compute the current error
            error = y_r - y;
            % differentiate error and y
            self.differentiateError(error);
            self.differentiateY(y);

            % PD Control
            if flag==true
                u_unsat = self.kp*error + self.kd*self.error_dot;
            else
                u_unsat = self.kp*error - self.kd*self.y_dot;
            end
            % return saturated control signal
            u = self.saturate(u_unsat);
        end
        %----------------------------
        function self = differentiateError(self, error)
            self.error_dot = self.beta*self.error_dot + (1-self.beta)*((error - self.error_d1) / self.Ts);
            self.error_d1 = error;            
        end
        %----------------------------
        function self = differentiateY(self, y)
            self.y_dot = self.beta*self.y_dot + (1-self.beta)*((y-self.y_d1)/self.Ts);
            self.y_d1 = y;
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







