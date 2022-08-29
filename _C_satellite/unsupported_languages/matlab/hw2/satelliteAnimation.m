classdef satelliteAnimation
    %
    %    Create satellite animation
    %
    %--------------------------------
    properties
        base_handle
        panel_handle
        length
        width
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = satelliteAnimation(P)
            self.length = P.length;
            self.width = P.width;
            
            figure(1), clf
            plot([-2*P.length,2*P.length],[0,0],'k--'); % plot track
            hold on
            % initialize the base and panel to initial conditions
            self=self.drawBase(P.theta0);
            self=self.drawPanel(P.phi0);
            axis([-2*P.length, 2*P.length, -2*P.length, 2*P.length]);
        end
        %---------------------------
        function self=update(self, x)
            % Draw satellite is the main function that will call the functions:
            % drawBase and drawPanel create the animation.
            % x is the system state
            theta = x(1);    % angle of base
            phi   = x(2);    % angle of panel

            self=self.drawBase(theta);
            self=self.drawPanel(phi);
            drawnow
        end
        %---------------------------
        function self=drawBase(self, theta)
            % define points on base (without rotation)
            pts = [...
                self.width/2, -self.width/2;...
                self.width/2, -self.width/6;...
                self.width/2+self.width/6, -self.width/6;...
                self.width/2+self.width/6,  self.width/6;...
                self.width/2, self.width/6;...
                self.width/2, self.width/2;...
                -self.width/2, self.width/2;...
                -self.width/2, self.width/6;...
                -self.width/2-self.width/6, self.width/6;...
                -self.width/2-self.width/6, -self.width/6;...
                -self.width/2, -self.width/6;...
                -self.width/2, -self.width/2;...
                ]';
            % define rotation matrix
            R = [cos(theta), sin(theta); -sin(theta), cos(theta)];
            % rotate points
            pts = R*pts;

            if isempty(self.base_handle)
                self.base_handle = fill(pts(1,:),pts(2,:),'b');
            else
                set(self.base_handle,'XData',pts(1,:));
                set(self.base_handle,'YData',pts(2,:));
            end
        end
        %---------------------------
        function self=drawPanel(self, phi)
            % define points on base (without rotation)
            pts = [...
                -self.length, -self.width/6;...
                self.length, -self.width/6;...
                self.length,  self.width/6;...
                -self.length,  self.width/6;...
                ]';
            % define rotation matrix
            R = [cos(phi), sin(phi); -sin(phi), cos(phi)];
            % rotate points
            pts = R*pts;
            
            if isempty(self.panel_handle)
                self.panel_handle = fill(pts(1,:),pts(2,:),'g');
            else
                set(self.panel_handle,'XData',pts(1,:));
                set(self.panel_handle,'YData',pts(2,:));
            end
        end
    end
end