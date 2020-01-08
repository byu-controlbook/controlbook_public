clear all

% Physical parameters of arm known to the controller
P.m = 0.5;  % kg
P.ell = 0.3; % m
P.b = 0.01; % N m s
P.g = 9.8; % m/s^2

% parameters for animation
P.length = 1;
P.width = 0.2;

% initial conditions
P.theta0 = 0;
P.thetadot0 = 0;

% input constraint
P.tau_max = 1;

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.1;   % the plotting and animation is updated at this rate


