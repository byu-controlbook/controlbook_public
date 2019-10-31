armParamHW11;  % load parameters

% instantiate arm, controller, and reference input classes 
addpath('../hw8'); arm = armDynamics(P);  
controller = armController(P);  
addpath('../hw_a'); reference = signalGenerator(30*pi/180, 0.05);  
addpath('../hw_a'); disturbance = signalGenerator(0.25, 0.0);

% instantiate the data plots and animation
addpath('../hw_a'); dataPlot = dataPlotter(P);
addpath('../hw_a'); animation = armAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = arm.h();
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        d = disturbance.step(t);
        n = 0;  % noise
        u = controller.update(r, y+n);  % Calculate the control value
        y = arm.update(u+d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(arm.state);
    dataPlot.update(t, r, arm.state, u);
end

