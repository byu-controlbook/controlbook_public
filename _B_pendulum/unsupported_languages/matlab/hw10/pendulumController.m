classdef pendulumController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        thetaCtrl
        filter_gain
        filter_pole
        filter_state
        Ts
    end
    %----------------------------
    methods
        %----------------------------
        function self = pendulumController(P)
            % Instantiates the SS_ctrl object
            self.zCtrl = PIDControl(P.kp_z, P.ki_z, P.kd_z, P.theta_max, P.beta, P.Ts);
            self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th, P.F_max, P.beta, P.Ts);
            self.filter_gain = -3.0/(2.0*P.ell*P.DC_gain);
            self.filter_pole = sqrt(3.0*P.g/(2.0*P.ell));
            self.filter_state = 0.0;
            self.Ts = P.Ts;
        end
        %----------------------------
        function F = update(self, z_r, output)
            z = output(1);
            theta = output(2);
            % the reference angle for theta comes from the outer loop PID control
            theta_r = self.zCtrl.PID(z_r, z, false);
            theta_r = self.zeroCancelingFilter(theta_r);
            % the force applied to the cart comes from the inner loop PD control
            F = self.thetaCtrl.PD(theta_r, theta, false);
        end
        %----------------------------
        function w = zeroCancelingFilter(self, v)
            self.filter_state = self.filter_state...
                + self.Ts * (-self.filter_pole * self.filter_state ...
                             + self.filter_gain * v);
            w = self.filter_state;
        end
    end
end