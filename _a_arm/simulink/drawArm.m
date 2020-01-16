function drawArm(u,P)

    % process inputs to function
    theta     = u(1);
    thetadot  = u(2);
    reference = u(3);
    torque =    u(4);
    t         = u(5);


    % define persistent variables 
    persistent link_handle
    
    persistent theta_history
    persistent theta_ref_history
    persistent torque_history
    persistent time_history
    
    persistent theta_ref_handle
    persistent theta_handle
    persistent torque_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        plot([0,P.length],[0,0],'k--'); % plot track
        hold on
        link_handle = drawLink(theta, P.length, P.width, []);
        axis([-2*P.length, 2*P.length, -2*P.length, 2*P.length]);
        
        figure(2), clf
        subplot(2, 1, 1)
        hold on
        theta_ref_handle = plot(0, 0, 'g');
        theta_handle    = plot(0, 0, 'b');
        ylabel('theta(deg)')
        title('Arm Data')
        subplot(2, 1, 2)
        hold on
        torque_handle    = plot(0, 0, 'b');
        ylabel('torque(N-m)')
        
        % at every other time step, redraw base and rod
    else
        drawLink(theta, P.length, P.width, link_handle);
        
        % update the time history of all plot variables
        time_history(end+1) = t;
        theta_ref_history(end+1) = 180/pi*reference;
        theta_history(end+1) = 180/pi*theta;
        torque_history(end+1) = torque;
        
        % update the plots with associated histories
        set(theta_ref_handle, 'Xdata', time_history, 'Ydata', theta_ref_history)
        set(theta_handle, 'Xdata', time_history, 'Ydata', theta_history)
        set(torque_handle, 'Xdata', time_history, 'Ydata', torque_history)
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
 