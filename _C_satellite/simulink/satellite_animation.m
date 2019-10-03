function satellite_animation(u, P)

    % process inputs to function
    theta    = u(1);
    phi      = u(2);
    %thetadot = u(3);
    %phidot   = u(4);
    t        = u(5);
    
    % define persistent variables 
    persistent base_handle
    persistent panel_handle
    
    % initialize plot and persistent vars at t=0
    if t==0
        figure(1), clf
        % plot track
        plot([-2*P.length,2*P.length],[0,0],...
            'k--'); 
        hold on
        base_handle...
            = drawBase(theta, P.width, []);
        panel_handle...
            = drawPanel(phi,P.width,P.length,[]);
        axis([-2*P.length, 2*P.length,...
              -2*P.length, 2*P.length]);
    
        
    % otherwise, redraw base and rod
    else 
        drawBase(theta, P.width, base_handle);
        drawPanel(phi, P.width, P.length, ...
                  panel_handle);
    end
end

   
%
%=============================================
% drawBase
% draw the base of the pendulum
% return handle if 3rd argument is empty, 
% otherwise use 3rd arg as handle
%=============================================
%
function handle = drawBase(theta, w, handle)

  % define points on base (without rotation)
  pts = [...
      w/2, -w/2;...
      w/2, -w/6;...
      w/2+w/6, -w/6;...
      w/2+w/6,  w/6;...
      w/2, w/6;...
      w/2, w/2;...
      -w/2, w/2;...
      -w/2, w/6;...
      -w/2-w/6, w/6;...
      -w/2-w/6, -w/6;...
      -w/2, -w/6;...
      -w/2, -w/2;...
      ]';
  % define rotation matrix
  R = [cos(theta), sin(theta);...
       -sin(theta), cos(theta)];
  % rotate points
  pts = R*pts;
  % break into X and Y components
  X = pts(1,:);
  Y = pts(2,:);

  if isempty(handle)
    handle = fill(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 
%
%=============================================
% drawPanel
% draw the solar panel
% return handle if 3rd argument is empty, 
% otherwise use 3rd arg as handle
%=============================================
%
function handle = drawPanel(phi, w, L, handle)

% define points on base (without rotation)
  pts = [...
      -L, -w/6;...
       L, -w/6;...
       L,  w/6;...
      -L,  w/6;...
      ]';
  % define rotation matrix
  R = [cos(phi), sin(phi);...
      -sin(phi), cos(phi)];
  % rotate points
  pts = R*pts;
  % break into X and Y components
  X = pts(1,:);
  Y = pts(2,:);

  if isempty(handle)
    handle = fill(X, Y, 'g');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

  