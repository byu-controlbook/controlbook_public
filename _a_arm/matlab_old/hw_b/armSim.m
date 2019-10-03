armParamHWB;  % load parameters

% instantiate arm, and reference input classes 
arm = armDynamics(P);  
reference = signalGenerator(0.01, 0.02);  
torque = signalGenerator(0.2, 0.05);

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = armAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        tau = torque.square(t);  % Calculate the input force
        arm.propagateDynamics(tau);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawArm(arm.states);
    dataPlot.updatePlots(t, ref_input, arm.states, tau);
end
