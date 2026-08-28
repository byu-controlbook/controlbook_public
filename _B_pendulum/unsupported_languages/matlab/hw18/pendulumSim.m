% load controller design
if 0
    loopshape_pendulum_in
    loopshape_pendulum_out
end

% instantiate pendulum, controller, and reference input classes 
alpha = 0.2;
addpath('../hw8'); pendulum = pendulumDynamics(alpha, P);  
controller = pendulumController(P);  
addpath('../hw2'); reference = signalGenerator(0.5, 0.02);  
addpath('../hw2'); disturbance = signalGenerator(0.1, 0);  

% instantiate the data plots and animation
addpath('../hw2'); dataPlot = dataPlotter(P);
addpath('../hw2'); animation = pendulumAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = pendulum.h();
while t < P.t_end  
    t_next_plot = t + P.t_plot;
    while t < t_next_plot 
        r = reference.square(t);
        d = disturbance.step(t);
        n = [0; 0];  % noise
        u = controller.update(r, y + n);  % Calculate the control value
        y = pendulum.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(pendulum.state);
    dataPlot.update(t, r, pendulum.state, u);
end
