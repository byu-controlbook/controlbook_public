pendulumParamHWB;  % load parameters

% instantiate pendulum, and reference input classes 
pendulum = pendulumDynamics(P);  
addpath('../hw_a'); reference = signalGenerator(0.5, 0.02);  
addpath('../hw_a'); force = signalGenerator(1, 1);

% instantiate the data plots and animation
addpath('../hw_a'); dataPlot = dataPlotter(P);
addpath('../hw_a'); animation = pendulumAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        f = force.sin(t);  % Calculate the input force
        y = pendulum.update(f);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(pendulum.state);
    dataPlot.update(t, r, pendulum.state, f);
end
