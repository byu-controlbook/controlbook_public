function drawArm(u,P)

    % process inputs to function
    theta    = u(1);
    thetadot = u(2);
    t        = u(3);


    % define persistent variables 
    persistent link_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        plot([0,P.length],[0,0],'k--'); % plot track
        hold on
        link_handle = drawLink(theta, P.length, P.width, []);
        axis([-2*P.length, 2*P.length, -2*P.length, 2*P.length]);
    
        
    % at every other time step, redraw base and rod
    else 
        drawLink(theta, P.length, P.width, link_handle);
    end
end

   
%
%=======================================================================
% drawLink
% draw the link
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawLink(theta, L, w, handle)
  
  pts = [ 0, L, L, 0; -w/2, -w/2, w/2, w/2];
  R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
  pts = R*pts;
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = fill(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 