P = loopshape_example;
P.t_start = 0;
P.t_end = 100;
P.Ts = 0.01;
P.t_plot = 0.1;

% instantiate arm, controller, and reference input classes 
plant = dynamics(P);  
ctrl = controller(P);  
reference = signalGenerator(1, 0.02);  
disturbance = signalGenerator(1, 0.0);
noise = signalGenerator(0.01);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);

% main simulation loop
t = P.t_start;  
y = plant.h();
while t < P.t_end  
    t_next_plot = t + P.t_plot;
    while t < t_next_plot 
        r = reference.square(t);
        d = disturbance.step(t);
        n = noise.random(t);  
        u = ctrl.update(r, y + n);  
        y = plant.update(u + d);  
        t = t + P.Ts; 
    end
    % update animation and data plots
    dataPlot.update(t, r, plant.state, u);
end
