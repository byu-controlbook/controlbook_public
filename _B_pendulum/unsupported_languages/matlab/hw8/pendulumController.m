classdef pendulumController  < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        kp_z
        kd_z
        kp_th
        kd_th
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
            self.kp_z = P.kp_z;
            self.kd_z = P.kd_z;
            self.kp_th = P.kp_th;
            self.kd_th = P.kd_th;
            self.filter_gain = -3.0/(2.0*P.ell*P.DC_gain);
            self.filter_pole = sqrt(3.0*P.g/(2.0*P.ell));
            self.filter_state = 0.0;
            self.Ts = P.Ts;
        end
        %----------------------------
        function F = update(self, z_r, state)
            z = state(1);
            theta = state(2);
            zdot = state(3);
            thetadot = state(4);
            % the reference angle for theta comes from the outer loop PD control
            tmp = self.kp_z * (z_r - z) - self.kd_z * zdot;
            theta_r = self.zeroCancelingFilter(tmp);
            % the force applied to the cart comes from the inner loop PD control
            F = self.kp_th * (theta_r - theta) - self.kd_th * thetadot;
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