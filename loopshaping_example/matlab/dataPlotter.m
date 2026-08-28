classdef dataPlotter < handle
    %    This class plots the time histories for the pendulum data.
    %----------------------------
    properties
        % data histories
        time_history
        ref_history
        y_history
        u_history
        index
        % figure handles
        ref_handle
        y_handle
        u_handle
    end
    methods
        %--constructor--------------------------
        function self = dataPlotter(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.ref_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.y_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.u_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(5), clf
            subplot(2, 1, 1)
                hold on
                self.ref_handle = plot(self.time_history, self.ref_history, 'g');
                self.y_handle    = plot(self.time_history, self.y_history, 'b');
                ylabel('y')
                title('System Data')
            subplot(2, 1, 2)
                hold on
                self.u_handle    = plot(self.time_history, self.u_history, 'b');
                ylabel('torque(N-m)')
        end
        %----------------------------
        function self = update(self, t, reference, states, ctrl)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.ref_history(self.index) = reference;
            self.y_history(self.index) = states(1);
            self.u_history(self.index) = ctrl;
            self.index = self.index + 1;

            % update the plots with associated histories
            set(self.ref_handle, 'Xdata', self.time_history, 'Ydata', self.ref_history)
            set(self.y_handle, 'Xdata', self.time_history, 'Ydata', self.y_history)
            set(self.u_handle, 'Xdata', self.time_history, 'Ydata', self.u_history)
        end
    end
end
