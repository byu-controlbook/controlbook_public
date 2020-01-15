% Physical parameters of the inverted pendulum known to the controller
P.m1 = 0.25;     % Mass of the pendulum, kg
P.m2 = 1.0;      % Mass of the cart, kg
P.ell = 0.5;    % Length of the rod, m
P.g = 9.8;       % Gravity, m/s**2
P.b = 0.05;      % Damping coefficient, Ns

% parameters for animation
P.w = 0.5;       % Width of the cart, m
P.h = 0.15;      % Height of the cart, m
P.gap = 0.005;   % Gap between the cart and x-axis
P.radius = 0.06; % Radius of circular part of pendulum

% Initial Conditions
P.z0 = 0.0;                % initial cart position, m
P.theta0 = 0.0*pi/180;     % initial rod angle, rads
P.zdot0 = 0.0;             % initial cart velocity, m/s
P.thetadot0 = 0.0;         % initial rod angular velocity, rads/s

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 40.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate
