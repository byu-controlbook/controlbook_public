classdef transferFunction < handle
    properties
        state
        Ts
        A
        B
        C
        D
    end
    methods
        function self = transferFunction(num, den, Ts)
            m = length(num)-1;
            n = length(den)-1;
            self.state = zeros(n,1);
            self.Ts = Ts;
            % make leading coef of den == 1
            if den(1)~= 1
                num = num/den(1);
                den = den/den(1);
            end
            % put tranfer function in control canonic form
            self.A = zeros(n,n);
            for i=1:n
                self.A(1,i) = -den(i+1);
            end
            for i=2:n
                self.A(i,i-1) = 1;
            end
            self.B = [1; zeros(n-1,1)];
            self.C = zeros(1,n);
            if m==n
                self.D = num(1);
                for i=n:-1:1
                    self.C(i) = num(i+1) - num(1)*den(i+1);
                end
            else
                self.D = 0;
                for i=n:-1:n-m+1
                    self.C(i) = num(i+1);
                end
            end
        end
        function y = update(self, u)
            % update transfer function using RK4 integration
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2*F1, u);
            F3 = self.f(self.state + self.Ts/2*F2, u);
            F4 = self.f(self.state + self.Ts*F3, u);
            self.state = self.state...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            y = self.C * self.state + self.D * u;
        end
        function xdot = f(self, x, u)
            xdot = self.A * x + self.B * u;
        end
    end
end