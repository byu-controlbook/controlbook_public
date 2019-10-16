armParamHW10;  % load parameters

% instantiate arm, controller, and reference input classes 
% Instantiate Dynamics class
addpath('../hw8'); arm = armDynamics(P);  
ctrl = armController(P);  
amplitude = 30*pi/180; % amplitude of reference input
frequency = 0.05; % frequency of reference input
addpath('../hw_a'); reference = signalGenerator(amplitude, frequency);  

% instantiate the data plots and animation
addpath('../hw_a'); dataPlot = plotData(P);
addpath('../hw_a'); animation = armAnimation(P);

% disturbance
d = 0.1;

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.update(ref_input, arm.output);  % Calculate the control value
        arm.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(arm.state);
    dataPlot.update(t, ref_input, arm.state, u);
end


