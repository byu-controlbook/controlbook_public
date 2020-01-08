% equalibrium angle and torque
P.theta_e = 0*pi/180;
P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e);

%  tuning parameters
%tr = 0.8; % part (a)
tr = 0.4;  % tuned to get fastest possible rise time before saturation.
zeta = 0.707;

% desired closed loop polynomial
wn = 2.2/tr;
Delta_cl_d = [1, 2*zeta*wn, wn^2];

% select PD gains
P.kp = Delta_cl_d(3)*(P.m*P.ell^2)/3;
P.kd = (P.m*P.ell^2)/3*(Delta_cl_d(2)-3*P.b/(P.m*P.ell^2));

