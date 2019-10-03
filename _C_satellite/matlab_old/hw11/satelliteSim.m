satelliteParamHW11  % load parameters

% instantiate satellite, and reference input classes 
satellite = satelliteDynamics(P);  
ctrl = satelliteController(P);  
amplitude = 15*pi/180; % amplitude of reference input
frequency = 0.02; % frequency of reference input
reference = signalGenerator(amplitude, frequency);  

% set disturbance input
disturbance = 0.0;%1.0;

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = satelliteAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        tau = ctrl.u(ref_input, satellite.outputs());  % Calculate the control value
        sys_input = tau+disturbance;  % input to plant is control input + disturbance
        satellite.propagateDynamics(sys_input);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawSatellite(satellite.states);
    dataPlot.updatePlots(t, ref_input, satellite.states, tau);
end


