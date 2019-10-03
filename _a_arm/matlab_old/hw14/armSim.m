armParamHW14;  % load parameters

% instantiate arm, controller, and reference input classes 
% Instantiate Dynamics class
arm = armDynamics(P);  
ctrl = armController(P);  
amplitude = 30*pi/180; % amplitude of reference input
frequency = 0.05; % frequency of reference input
reference = signalGenerator(amplitude, frequency);  

% set disturbance input
disturbance = 0.01;

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = armAnimation(P);
observerPlot = plotObserverData(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, arm.outputs());  % Calculate the control value
        sys_input = u+disturbance;  % input to plant is control input + disturbance
        arm.propagateDynamics(sys_input);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawArm(arm.states);
    dataPlot.updatePlots(t, ref_input, arm.states, u);
    observerPlot.updatePlots(t, arm.states, ctrl.x_hat);
end


