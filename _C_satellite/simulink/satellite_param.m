clear all

% Physical parameters of the satellite known to the controller
P.Js = 5; % kg m^2
P.Jp = 1;  % kg m^2
% P.k = 0.15; % N m 
P.k = 0.1; % N m 
P.b = 0.05; % N m s

% parameters for animation
P.length = 1;  % length of solar panel
P.width = .3; % width of satellite body

% Initial Conditions
P.theta0    = 0;
P.phi0      = 0;
P.thetadot0 = 0;
P.phidot0   = 0;

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% maximum torque
P.taumax = 5; %Nm

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain
