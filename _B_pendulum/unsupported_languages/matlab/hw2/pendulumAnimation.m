classdef pendulumAnimation
    properties
        cart_handle
        rod_handle
        bob_handle
        ell
        width
        height
        gap
        radius
    end
    methods
        %------constructor-----------
        function self = pendulumAnimation(P)
            self.ell = P.ell;
            self.width = P.w;
            self.height = P.h;
            self.gap = P.gap;
            self.radius = P.radius;
            
            figure(1), clf
            % draw ground track
            plot([-2*self.ell, 2*self.ell],[0,0],'k'); 
            hold on
            % initialize the cart, rod, and bob 
            self=self.draw_cart(P.z0);
            self=self.draw_rod(P.z0, P.theta0);
            self=self.draw_bob(P.z0, P.theta0);
            % Set the x,y axis limits
            axis([-3*self.ell, 3*self.ell, -0.1, 3*self.ell]); 
            xlabel('z'); % label x-axis
        end

        function self=update(self, state)
            % external method to update the animation
            z= state(1); % Horizontal position of cart, m
            theta = state(2);  % Angle of pendulum, rads
            self=self.draw_cart(z);
            self=self.draw_rod(z, theta);
            self=self.draw_bob(z, theta);
            drawnow
        end

        function self=draw_cart(self, z)
            % specify X-Y locations of corners of the cart
            X = [z-self.width/2, z+self.width/2,...
                 z+self.width/2, z-self.width/2];
            Y = [self.gap, self.gap,...
                 self.gap+self.height, self.gap+self.height];
            % plot or update cart
            if isempty(self.cart_handle)
                self.cart_handle = fill(X,Y,'b');
            else
                set(self.cart_handle,'XData',X);
            end
        end

        function self=draw_rod(self, z, theta)
            % specify X-Y locations of ends or rod
            X = [z, z+self.ell*sin(theta)]; 
            Y = [...
                self.gap+self.height,...
                self.gap + self.height + self.ell*cos(theta)...
                ]; 
            % plot or update rod
            if isempty(self.rod_handle)
                self.rod_handle = plot(X, Y, 'k');
            else
                set(self.rod_handle,'XData', X, 'YData', Y);
            end
        end

        function self=draw_bob(self, z, theta)
            th = 0:2*pi/10:2*pi;
            % center of bob
            center = [z + (self.ell+self.radius)*sin(theta) ...
                self.gap+self.height + ...
                (self.ell+self.radius)*cos(theta)];
            % points that define the bob
            X = center(1) + self.radius*cos(th);
            Y = center(2) + self.radius*sin(th);
            % plot or update bob
            if isempty(self.bob_handle)
                self.bob_handle = fill(X,Y,'g');
            else
                set(self.bob_handle,'XData',X);
                set(self.bob_handle,'YData',Y);
            end
        end 
    end
end