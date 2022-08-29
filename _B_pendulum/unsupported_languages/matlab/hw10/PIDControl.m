classdef PIDControl < handle
    %----------------------------
    properties
        kp
        ki
        kd
        limit
        beta
        Ts
        init_flag
        y_dot
        y_d1
        error_dot
        error_d1
        integrator
    end
    %----------------------------
    methods
        %----------------------------
        function self = PIDControl(kp, ki, kd, limit, beta, Ts)
            self.kp = kp;                 % Proportional control gain
            self.ki = ki;                 % Integral control gain
            self.kd = kd;                 % Derivative control gain
            self.limit = limit;           % The output will saturate at this limit
            self.beta = beta;
            self.Ts = Ts;

            self.init_flag = 1;           % true for first time step
            self.y_dot = 0.0;             % estimated derivative of y
            self.y_d1 = 0.0;              % Signal y delayed by one sample
            self.error_dot = 0.0;         % estimated derivative of error
            self.error_d1 = 0.0;          % Error delayed by one sample
            self.integrator = 0.0;        % value of the integrator
        end
         %----------------------------
        function u_sat = PID(self, y_r, y, flag)
            %
            %    PID control,
            %    
            %    if flag==True, then returns
            %        u = kp*error + ki*integral(error) + kd*error_dot.
            %    else returns 
            %        u = kp*error + ki*integral(error) - kd*y_dot.
            %    
            %    error_dot and y_dot are computed numerically using a dirty derivative
            %    the integral is computed numerically
            %

            % Compute the current error
            error = y_r - y;
            % integrate error
            self.integrator = self.integrator...
                + (self.Ts/2)*(error+self.error_d1);
            % initialize differentiators.
            if self.init_flag==1
                self.y_d1 = y;
                self.error_d1 = error;
                self.init_flag=0;
            end
            % PID Control
            if flag==true
                % differentiate error
                self.error_dot = self.beta*self.error_dot...
                    + (1-self.beta)*((error - self.error_d1) / self.Ts);
                % PID control
                u_unsat = self.kp*error...
                        + self.ki*self.integrator...
                        + self.kd*self.error_dot;
            else
                % differentiate y
                self.y_dot = self.beta*self.y_dot...
                    + (1-self.beta)*((y-self.y_d1)/self.Ts);
                % PID control
                u_unsat = self.kp*error...
                        + self.ki*self.integrator...
                        - self.kd*self.y_dot;
            end
            % return saturated control signal
            u_sat = self.saturate(u_unsat);
            % integrator anti-windup
            if self.ki~=0
                self.integrator = self.integrator...
                    + self.Ts/self.ki*(u_sat-u_unsat);
            end
            % update delayed variables
            self.error_d1 = error;
            self.y_d1 = y;
        end
       %----------------------------
        function u_sat = PD(self, y_r, y, flag)
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
            % initialize differentiators.
            if self.init_flag==1
                self.y_d1 = y;
                self.error_d1 = error;
                self.init_flag=0;
            end
            if flag==true
                % differentiate error
                self.error_dot = self.beta*self.error_dot...
                    + (1-self.beta)*((error - self.error_d1) / self.Ts);
                % PD control
                u_unsat = self.kp*error + self.kd*self.error_dot;
            else
                % differentiate y
                self.y_dot = self.beta*self.y_dot...
                    + (1-self.beta)*((y-self.y_d1)/self.Ts);
                % PD control                
                u_unsat = self.kp*error - self.kd*self.y_dot;
            end
            % return saturated control signal
            u_sat = self.saturate(u_unsat);
            % update delayed variables
            self.error_d1 = error;
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







