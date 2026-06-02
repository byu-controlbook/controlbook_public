satelliteParamHW10  % load parameters

% instantiate satellite, and reference input classes 
alpha = 0.2;
addpath('../hw3'); satellite = satelliteDynamics(alpha, P);  
controller = satelliteController(P);  
addpath('../hw2'); reference = signalGenerator(15*pi/180, 0.015);  
addpath('../hw2'); disturbance = signalGenerator(1.0, 0);  

% instantiate the data plots and animation
addpath('../hw2'); dataPlot = dataPlotter(P);
addpath('../hw2'); animation = satelliteAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = satellite.h();
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        d = disturbance.step(t);
        n = [0; 0];  % sensor noise
        u = controller.update(r, y + n);  
        y = satellite.update(u + d);
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(satellite.state);
    dataPlot.update(t, r, satellite.state, t);
end


