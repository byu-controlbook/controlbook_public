classdef armAnimation < handle
    properties
        link_handle
        length
        width
    end
    methods
        %------constructor-----------
        function self = armAnimation(P)
            self.length = P.length;
            self.width = P.width;
            
            figure(1), clf
            plot([0,self.length],[0,0],'k--'); % plot track
            hold on
            self.update([P.theta0; P.thetadot0]);
            axis([-2*self.length, 2*self.length,...
                  -2*self.length, 2*self.length]);
        end
        
        function self = update(self, x)
            theta = x(1);
            pts = [...
                0, -self.width/2;...
                self.length, -self.width/2;...
                self.length, self.width/2;...
                0, self.width/2;...
                ]';
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            pts = R*pts;
  
            if isempty(self.link_handle)
                self.link_handle = fill(pts(1,:), pts(2,:), 'b');
            else
                set(self.link_handle, 'XData', pts(1,:),...
                                      'YData', pts(2,:));
                drawnow
            end
        end
    end
end