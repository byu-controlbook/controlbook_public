function pendulumAnimation(u)

    % process inputs to function
    z        = u(1);
    theta    = u(2);
    zdot     = u(3);
    thetadot = u(4);
    t        = u(5);
    
    % drawing parameters
    L = 1;
    gap = 0.01;
    width = 1.0;
    height = 0.3;
    
    % define persistent variables 
    persistent base_handle
    persistent rod_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        track_width=2;
        plot([-track_width,track_width],[0,0],'k'); % plot track
        hold on
        base_handle = drawBase(z, width, height, gap, []);
        rod_handle  = drawRod(z, theta, L, gap, height, []);
        axis([-track_width, track_width, -L, 2*track_width-L]);
    
        
    % at every other time step, redraw base and rod
    else 
        drawBase(z, width, height, gap, base_handle);
        drawRod(z, theta, L, gap, height, rod_handle);
    end
end

   
%
%=======================================================================
% drawBase
% draw the base of the pendulum
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function new_handle = drawBase(z, width, height, gap, handle)
  
  pts = [...
      z-width/2, gap;...
      z+width/2, gap;...
      z+width/2, gap+height;...
      z-width/2, gap+height;...
      ];

  if isempty(handle)
    new_handle = fill(pts(:,1),pts(:,2),'b');
  else
    set(handle,'XData',pts(:,1));
    drawnow
  end
end
 
%
%=======================================================================
% drawRod
% draw the pendulum rod
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawRod(z, theta, L, gap, height, handle)

  
  X = [z, z+L*sin(theta)];
  Y = [gap+height, gap + height + L*cos(theta)];

  if isempty(handle)
    handle = plot(X, Y, 'k', 'LineWidth', 4);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

  