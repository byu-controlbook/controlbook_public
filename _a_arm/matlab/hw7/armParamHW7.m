% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
armParam % general arm parameters

% equalibrium angle and torque
P.theta_e = 0*pi/180;
P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e);

% PD gains
P.kp = 0.18;
P.kd = 0.095;

fprintf('\t kp: %f\n', P.kp)
fprintf('\t kd: %f\n', P.kd)



