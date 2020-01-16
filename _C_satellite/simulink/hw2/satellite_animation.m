function satellite_animation(u, P)
    % process inputs to function
    theta     = u(1);
    phi       = u(2);
    thetadot  = u(3);
    phidot    = u(4);
    reference = u(5);
    torque    = u(6);
    t         = u(7);
    
    % define persistent variables 
    persistent base_handle
    persistent panel_handle
    
    persistent time_history
    persistent theta_history
    persistent phi_history
    persistent phi_ref_history
    persistent torque_history
    
    persistent phi_ref_handle
    persistent phi_handle
    persistent theta_handle
    persistent torque_handle
    
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
          
          figure(2), clf
          % plot data
%           subplot(3, 1, 1)
%           hold on
%           phi_ref_handle = plot(time_history, phi_ref_history, 'g');
%           phi_handle    = plot(time_history, phi_history, 'b');
%           ylabel('phi(deg)')
%           title('Satellite Data')
%           subplot(3, 1, 2)
%           hold on
%           theta_handle    = plot(time_history, theta_history, 'b');
%           ylabel('theta(deg)')
%           subplot(3, 1, 3)
%           hold on
%           torque_handle    = plot(time_history, torque_history, 'b');
%           ylabel('torque(Nm)')

          subplot(3, 1, 1)
          hold on
          phi_ref_handle = plot(0, 0, 'g'); % First plot has to have some values, and at this point time_history, theta_history, etc, are all empty.
          phi_handle    = plot(0, 0, 'b');
          ylabel('phi(deg)')
          title('Satellite Data')
          subplot(3, 1, 2)
          hold on
          theta_handle    = plot(0, 0, 'b');
          ylabel('theta(deg)')
          subplot(3, 1, 3)
          hold on
          torque_handle    = plot(0, 0, 'b');
          ylabel('torque(Nm)')
          
          
        
    else 
        drawBase(theta, P.width, base_handle);
        drawPanel(phi, P.width, P.length, ...
            panel_handle);
        
        time_history(end+1) = t;
        phi_ref_history(end+1) = 180/pi*reference;
        phi_history(end+1) = 180/pi*phi;
        theta_history(end+1) = 180/pi*theta;
        torque_history(end+1) = torque;
        
        % update the plots with associated histories
        set(phi_ref_handle, 'Xdata', time_history, 'Ydata', phi_ref_history)
        set(phi_handle, 'Xdata', time_history, 'Ydata', phi_history)
        set(theta_handle, 'Xdata', time_history, 'Ydata', theta_history)
        set(torque_handle, 'Xdata', time_history, 'Ydata', torque_history)
        drawnow
    end
end
   
%=============================================
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
 
%=============================================
function handle = drawPanel(phi, w, L, handle)
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

  