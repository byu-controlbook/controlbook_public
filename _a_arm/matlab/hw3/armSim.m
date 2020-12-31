armParamHW3;  % load parameters

% instantiate arm, and reference input classes 
alpha = 0.0;  % parameter uncertainty coefficient
arm = armDynamics(alpha, P);  
addpath('../hw2'); reference = signalGenerator(0.01, 0.02);  
addpath('../hw2'); torque = signalGenerator(0.2, 0.05);

% instantiate the data plots and animation
addpath('../hw2'); dataPlot = dataPlotter(P);
addpath('../hw2'); animation = armAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics at rate Ts
    t_next_plot = t + P.t_plot;
    while t < t_next_plot 
        r = reference.square(t);
        u = torque.square(t);  % Calculate the input force
        y = arm.update(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots at rate t_plot
    animation.update(arm.state);
    dataPlot.update(t, r, arm.state, u);
end
