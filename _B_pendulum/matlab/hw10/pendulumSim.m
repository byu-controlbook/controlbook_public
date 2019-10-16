pendulumParamHW10;  % load parameters

% instantiate pendulum, controller, and reference input classes 
% Instantiate Dynamics class
addpath('../hw8'); pendulum = pendulumDynamics(P);  
ctrl = pendulumController(P);  
amplitude = 0.5; % amplitude of reference input
frequency = 0.02; % frequency of reference input
addpath('../hw_a'); reference = signalGenerator(amplitude, frequency);  

% instantiate the data plots and animation
addpath('../hw_a'); dataPlot = plotData(P);
addpath('../hw_a'); animation = pendulumAnimation(P);

% disturbance 
d = 0.0;

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.update(ref_input, pendulum.output);  % Calculate the control value
        pendulum.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(pendulum.state);
    dataPlot.update(t, ref_input, pendulum.state, u);
end


