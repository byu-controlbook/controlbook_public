classdef plotData < handle
    %    This class plots the time histories for the satellite data.
    %----------------------------
    properties
        % data histories
        time_history
        phi_ref_history
        phi_history
        theta_history
        torque_history
        index
        % figure handles
        phi_ref_handle
        phi_handle
        theta_handle
        torque_handle
    end
    methods
        %--constructor--------------------------
        function self = plotData(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.phi_ref_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.phi_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.torque_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(2), clf
            subplot(3, 1, 1)
                hold on
                self.phi_ref_handle = plot(self.time_history, self.phi_ref_history, 'g');
                self.phi_handle    = plot(self.time_history, self.phi_history, 'b');
                ylabel('phi(deg)')
                title('Satellite Data')
            subplot(3, 1, 2)
                hold on
                self.theta_handle    = plot(self.time_history, self.theta_history, 'b');
                ylabel('theta(deg)')
            subplot(3, 1, 3)
                hold on
                self.torque_handle    = plot(self.time_history, self.torque_history, 'b');
                ylabel('torque(Nm)')
        end
        %----------------------------
        function self = updatePlots(self, t, reference, states, ctrl)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.phi_ref_history(self.index) = 180/pi*reference;
            self.phi_history(self.index) = 180/pi*states(2);
            self.theta_history(self.index) = 180/pi*states(3);
            self.torque_history(self.index) = ctrl;
            self.index = self.index + 1;

            % update the plots with associated histories
            set(self.phi_ref_handle, 'Xdata', self.time_history, 'Ydata', self.phi_ref_history)
            set(self.phi_handle, 'Xdata', self.time_history, 'Ydata', self.phi_history)
            set(self.theta_handle, 'Xdata', self.time_history, 'Ydata', self.theta_history)
            set(self.torque_handle, 'Xdata', self.time_history, 'Ydata', self.torque_history)
        end
    end
end
