satelliteParamHWA  % load parameters

% instantiate reference input classes 
reference = signalGenerator(0.5, 0.1);
thetaRef = signalGenerator(2*pi, 0.01);   
phiRef = signalGenerator(0.5, 0.1);
tauRef = signalGenerator(5, 0.5);


% instantiate the data plots and animation
dataPlot = plotData(P);
animation = satelliteAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.square(t);
    theta = thetaRef.sin(t);
    phi = phiRef.sin(t);
    tau = tauRef.sawtooth(t);
    % update animation and data plot
    state = [theta; phi; 0.0; 0.0];
    animation.update(state);
    dataPlot.update(t, r, state, tau);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.1)
end


