% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative

% equalibrium torque
P.theta_e = 0*pi/180;
P.tau_e = P.m*P.g*P.ell/2*cos(P.theta_e);

% PD gains
P.kp = 0.18;
P.kd = 0.095;


