pendulumParamHW14;  % load parameters

% instantiate pendulum, controller, and reference input classes 
alpha = 0.2;
addpath('../hw8'); pendulum = pendulumDynamics(alpha, P);  
controller = pendulumController(P);  
addpath('../hw2'); reference = signalGenerator(0.5, 0.02);  
addpath('../hw2'); disturbance = signalGenerator(0.1, 0);  
addpath('../hw2'); noise_z = signalGenerator(0.01);
addpath('../hw2'); noise_th = signalGenerator(0.01);

% instantiate the data plots and animation
addpath('../hw2'); dataPlot = dataPlotter(P);
addpath('../hw2'); animation = pendulumAnimation(P);
addpath('../hw13'); dataPlotObserver = dataPlotterObserver(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = pendulum.h();
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        d = disturbance.step(t);
        n = [noise_z.random(t); noise_th.random(t)];  % noise
        [u, xhat, dhat] = controller.update(r, y + n);  % Calculate the control value
        y = pendulum.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(pendulum.state);
    dataPlot.update(t, r, pendulum.state, u);
    dataPlotObserver.update(t, pendulum.state, xhat, d, dhat);
end
