classdef controller < handle
    properties
        control
        prefilter
    end
    methods
        function self = controller(P)
            self.prefilter = transferFunction(P.num_F, P.den_F, P.Ts);
            self.control = transferFunction(P.num_C, P.den_C, P.Ts);
        end
        function u = update(self, r, y)
            % prefilter the reference
            r_filtered = self.prefilter.update(r);

            % define error and update the controller
            error = r_filtered - y;
            u = self.control.update(error);
        end       
    end
end