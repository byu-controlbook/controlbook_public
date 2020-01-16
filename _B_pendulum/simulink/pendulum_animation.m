function pendulumAnimation(u)

% process inputs to function
z         = u(1);
theta     = u(2);
zdot      = u(3);
thetadot  = u(4);
reference = u(5);
ctrl      = u(6);
t         = u(7);

% drawing parameters
L = 1;
gap = 0.01;
width = 1.0;
height = 0.3;

% define persistent variables
persistent base_handle
persistent rod_handle

persistent z_history
persistent theta_history
persistent z_ref_history
persistent force_history
persistent time_history

persistent z_handle
persistent theta_handle
persistent z_ref_handle
persistent force_handle

% first time function is called, initialize plot and persistent vars
if t==0
    figure(1), clf
    track_width=2;
    plot([-track_width,track_width],[0,0],'k'); % plot track
    hold on
    base_handle = drawBase(z, width, height, gap, []);
    rod_handle  = drawRod(z, theta, L, gap, height, []);
    axis([-track_width, track_width, -L, 2*track_width-L]);
    
    % Create figure and axes handles
    figure(2), clf
    subplot(3, 1, 1)
    hold on
    z_ref_handle = plot(0, 0, 'g');
    z_handle    = plot(0, 0, 'b');
    ylabel('z(m)')
    title('Pendulum Data')
    subplot(3, 1, 2)
    hold on
    theta_handle    = plot(0, 0, 'b');
    ylabel('theta(deg)')
    subplot(3, 1, 3)
    hold on
    force_handle    = plot(0, 0, 'b');
    ylabel('force(N)')
    
    % at every other time step, redraw base and rod
else
    drawBase(z, width, height, gap, base_handle);
    drawRod(z, theta, L, gap, height, rod_handle);
    
    time_history(end+1) = t;
    z_ref_history(end+1) = reference;
    z_history(end+1) = z;
    theta_history(end+1) = 180/pi*theta;
    force_history(end+1) = ctrl;
    
    % update the plots with associated histories
    set(z_ref_handle, 'Xdata', time_history, 'Ydata', z_ref_history)
    set(z_handle, 'Xdata', time_history, 'Ydata', z_history)
    set(theta_handle, 'Xdata', time_history, 'Ydata', theta_history)
    set(force_handle, 'Xdata', time_history, 'Ydata', force_history)
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

  