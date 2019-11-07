satelliteParamHW13  % load parameters

% instantiate satellite, controller, and reference input classes 
addpath('../hw8'); satellite = satelliteDynamics(P);  
controller = satelliteController(P);  
addpath('../hw_a'); reference = signalGenerator(15*pi/180, 0.02);  
addpath('../hw_a'); disturbance = signalGenerator(1, 0);  
addpath('../hw_a'); noise_phi = signalGenerator(0.01);
addpath('../hw_a'); noise_th = signalGenerator(0.01);

% instantiate the data plots and animation
addpath('../hw_a'); dataPlot = dataPlotter(P);
addpath('../hw_a'); animation = satelliteAnimation(P);
addpath('../hw13'); dataPlotObserver = dataPlotterObserver(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = satellite.h();
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        d = disturbance.step(t);
        n = [noise_phi.random(t); noise_th.random(t)];  % noise
        [u, xhat] = controller.update(r, y + n);  % Calculate the control value
        y = satellite.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(satellite.state);
    dataPlot.update(t, r, satellite.state, u);
    dataPlotObserver.update(t, satellite.state, xhat);
end


