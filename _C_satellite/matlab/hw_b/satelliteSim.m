satelliteParamHWB  % load parameters

% instantiate satellite, and reference input classes 
satellite = satelliteDynamics(P);  
addpath('../hw_a'); reference = signalGenerator(0.5, 0.1);
addpath('../hw_a'); torque = signalGenerator(.1, .1);

% instantiate the data plots and animation
addpath('../hw_a'); dataPlot = plotData(P);
addpath('../hw_a'); animation = satelliteAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        tau = torque.sin(t);  % Calculate the input force
        satellite.update(tau);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(satellite.state);
    dataPlot.update(t, ref_input, satellite.state, tau);
end


