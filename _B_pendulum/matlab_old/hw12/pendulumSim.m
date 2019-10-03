pendulumParamHW12;  % load parameters

% instantiate pendulum, controller, and reference input classes 
% Instantiate Dynamics class
pendulum = pendulumDynamics(P);  
ctrl = pendulumController(P);  
    amplitude = 0.5; % amplitude of reference input
    frequency = 0.05; % frequency of reference input
reference = signalGenerator(amplitude, frequency);  

% set disturbance input
disturbance = 0.5;

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = pendulumAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, pendulum.outputs());  % Calculate the control value
        sys_input = u+disturbance;  % input to plant is control input + disturbance
        pendulum.propagateDynamics(sys_input);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawPendulum(pendulum.states);
    dataPlot.updatePlots(t, ref_input, pendulum.states, u);
end


