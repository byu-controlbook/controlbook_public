classdef dataPlotter < handle
    %    This class plots the time histories for the pendulum data.
    %----------------------------
    properties
        % data histories
        time_history
        r_history
        y_history
        e1_history
        e2_history
        d_history
        dhat_history
        index
        % figure handles
        r_handle
        y_handle
        e1_handle
        e2_handle
        d_handle
        dhat_handle
    end
    methods
        %--constructor--------------------------
        function self = dataPlotter(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.r_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);            
            self.y_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.e1_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.e2_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.d_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.dhat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(2), clf
            subplot(3, 1, 1)
                hold on
                self.r_handle = plot(self.time_history, self.r_history, 'g');
                self.y_handle    = plot(self.time_history, self.y_history, 'b');
                ylabel('y')
                title('Output')
            subplot(3, 1, 2)
                hold on
                self.e1_handle    = plot(self.time_history, self.e1_history, 'b');
                self.e2_handle    = plot(self.time_history, self.e2_history, 'g');
                ylabel('estimation error')
            subplot(3, 1, 3)
                hold on
                self.d_handle    = plot(self.time_history, self.d_history, 'b');
                self.dhat_handle    = plot(self.time_history, self.dhat_history, 'g');
                ylabel('disturbance and estimate')
        end
        %----------------------------
        function self = update(self, t, r, y, x, xhat, d, dhat)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.r_history(self.index) = r;
            self.y_history(self.index) = y;
            self.e1_history(self.index) = x(1)-xhat(1);
            self.e2_history(self.index) = x(2)-xhat(2);
            self.d_history(self.index) = d;
            self.dhat_history(self.index) = dhat;
            self.index = self.index + 1;

            % update the plots with associated histories
            set(self.r_handle, 'Xdata', self.time_history, 'Ydata', self.r_history)
            set(self.y_handle, 'Xdata', self.time_history, 'Ydata', self.y_history)
            set(self.e1_handle, 'Xdata', self.time_history, 'Ydata', self.e1_history)
            set(self.e2_handle, 'Xdata', self.time_history, 'Ydata', self.e2_history)
            set(self.d_handle, 'Xdata', self.time_history, 'Ydata', self.d_history)
            set(self.dhat_handle, 'Xdata', self.time_history, 'Ydata', self.dhat_history)
        end
    end
end
