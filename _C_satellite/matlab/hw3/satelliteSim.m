satelliteParamHWB  % load parameters

% instantiate satellite, and reference input classes 
satellite = satelliteDynamics(P);  
addpath('../hw2'); reference = signalGenerator(0.5, 0.1);
addpath('../hw2'); torque = signalGenerator(.1, .1);

% instantiate the data plots and animation
addpath('../hw2'); dataPlot = dataPlotter(P);
addpath('../hw2'); animation = satelliteAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        u = torque.sin(t);  % Calculate the input force
        y = satellite.update(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(satellite.state);
    dataPlot.update(t, r, satellite.state, u);
end


