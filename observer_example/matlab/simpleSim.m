simpleParam;  % load parameters

% instantiate system, controller, and reference input classes 
system = simpleDynamics(P);  
controller = simpleController(P);  
reference = signalGenerator(1, 0.05);  
disturbance = signalGenerator(1.0, 0.0);
noise = signalGenerator(0.01);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);

% main simulation loop
t = P.t_start;  
y = system.h();
while t < P.t_end  
    t_next_plot = t + P.t_plot;
    while t < t_next_plot 
        r = reference.square(t);
        d = disturbance.step(t);
        n = 0;%noise.random(t);  
        [u, xhat, dhat] = controller.update(r, y + n);  
        y = system.update(u+d);  
        t = t + P.Ts; % advance time by Ts
    end
    % update data plots
    dataPlot.update(t, r, y, system.state, xhat, d, dhat);
end

