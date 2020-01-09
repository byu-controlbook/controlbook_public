classdef dataPlotter < handle
    %    This class plots the time histories for the pendulum data.
    %----------------------------
    properties
        % data histories
        time_history
        zref_history
        z_history
        theta_history
        force_history
        index
        % figure handles
        zref_handle
        z_handle
        theta_handle
        force_handle
    end
    methods
        %--constructor--------------------------
        function self = dataPlotter(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.zref_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.force_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(2), clf
            subplot(3, 1, 1)
                hold on
                self.zref_handle = plot(self.time_history, self.zref_history, 'g');
                self.z_handle    = plot(self.time_history, self.z_history, 'b');
                ylabel('z(m)')
                title('Pendulum Data')
            subplot(3, 1, 2)
                hold on
                self.theta_handle    = plot(self.time_history, self.theta_history, 'b');
                ylabel('theta(deg)')
            subplot(3, 1, 3)
                hold on
                self.force_handle    = plot(self.time_history, self.force_history, 'b');
                ylabel('force(N)')
        end
        %----------------------------
        function self = update(self, t, reference, states, ctrl)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.zref_history(self.index) = reference;
            self.z_history(self.index) = states(1);
            self.theta_history(self.index) = 180/pi*states(2);
            self.force_history(self.index) = ctrl;
            self.index = self.index + 1;

            % update the plots with associated histories
            set(self.zref_handle, 'Xdata', self.time_history, 'Ydata', self.zref_history)
            set(self.z_handle, 'Xdata', self.time_history, 'Ydata', self.z_history)
            set(self.theta_handle, 'Xdata', self.time_history, 'Ydata', self.theta_history)
            set(self.force_handle, 'Xdata', self.time_history, 'Ydata', self.force_history)
        end
    end
end
