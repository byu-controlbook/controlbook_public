classdef pendulumAnimation
    %
    %    Create pendulum animation
    %
    %--------------------------------
    properties
        base_handle
        rod_handle
        bob_handle
        ell
        width
        height
        gap
        radius
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = pendulumAnimation(P)
            self.ell = P.ell;
            self.width = P.w;
            self.height = P.h;
            self.gap = P.gap;
            self.radius = P.radius;
            
            figure(1), clf
            plot([-2*self.ell, 2*self.ell],[0,0],'k'); % draw track
            hold on
            % initialize the base, rod, and bob to initial conditions
            self=self.drawBase(P.z0);
            self=self.drawRod(P.z0, P.theta0);
            self=self.drawBob(P.z0, P.theta0);
            axis([-3*self.ell, 3*self.ell, -0.1, 3*self.ell]); % Change the x,y axis limits
            xlabel('z'); % label x-axis
        end
        %---------------------------
        function self=drawPendulum(self, x)
            % Draw pendulum is the main function that will call the functions:
            % drawCart, drawCircle, and drawRod to create the animation.
            % x is the system state
            z= x(1);        % Horizontal position of cart, m
            theta = x(2);   % Angle of pendulum, rads

            self=self.drawBase(z);
            self=self.drawRod(z, theta);
            self=self.drawBob(z, theta);
            drawnow
        end
        %---------------------------
        function self=drawBase(self, z)
            pts = [...
                z-self.width/2, self.gap;...
                z+self.width/2, self.gap;...
                z+self.width/2, self.gap+self.height;...
                z-self.width/2, self.gap+self.height;...
                ];

            if isempty(self.base_handle)
                self.base_handle = fill(pts(:,1),pts(:,2),'b');
            else
                set(self.base_handle,'XData',pts(:,1));
            end
        end
        %---------------------------
        function self=drawRod(self, z, theta)
            X = [z, z+self.ell*sin(theta)]; % X data points
            Y = [...
                self.gap+self.height,...
                self.gap + self.height + self.ell*cos(theta)...
                ]; % Y data points

            if isempty(self.rod_handle)
                self.rod_handle = plot(X, Y, 'k');
            else
                set(self.rod_handle,'XData', X, 'YData', Y);
            end
        end
        %---------------------------
        function self=drawBob(self, z, theta)
            th = 0:2*pi/10:2*pi;
            center = [...
                z + (self.ell+self.radius)*sin(theta),...
                self.gap+self.height+(self.ell+self.radius)*cos(theta)...
                ];
            pts = center + [self.radius*cos(th)', self.radius*sin(th)'];

            if isempty(self.bob_handle)
                self.bob_handle = fill(pts(:,1),pts(:,2),'g');
            else
                set(self.bob_handle,'XData',pts(:,1));
                set(self.bob_handle,'YData',pts(:,2));
            end
        end 
    end
end