armParamHW7;  % load parameters

% instantiate arm, controller, and reference input classes 
% Instantiate Dynamics class
alpha=0;
addpath('../hw3'); arm = armDynamics(alpha,P);
controller = armController(P);  
addpath('../hw2'); reference = signalGenerator(30*pi/180, 0.05); 
addpath('../hw2'); disturbance = signalGenerator(0.0, 0.0);

% instantiate the data plots and animation
addpath('../hw2'); dataPlot = dataPlotter(P);
addpath('../hw2'); animation = armAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = arm.h();
while t < P.t_end  
    t_next_plot = t + P.t_plot;
    % Propagate dynamics in between plot samples
    while t < t_next_plot 
        r = reference.square(t);
        d = disturbance.step(t);
        x = arm.state;  % use the state and not the output
        u = controller.update(r, x);  % Calculate the control value
        y = arm.update(u+d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(arm.state);
    dataPlot.update(t, r, arm.state, u);
end


