
% sample rate
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;

% equalibrium torque
P.theta_e = 0*pi/180;
P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e);

% saturation constraint
P.tau_max = 1;
tau_max = P.tau_max-P.tau_e;

%  tuning parameters
%tr = 0.8; % part (a)
tr = 0.4;  % tuned to get fastest possible rise time before saturation.
zeta = 0.707;
P.ki = 0.1;  % integrator gain

% desired closed loop polynomial
wn = 2.2/tr;
Delta_cl_d = [1, 2*zeta*wn, wn^2];

% select PD gains
P.kp = Delta_cl_d(3)*(P.m*P.ell^2)/3;
P.kd = (P.m*P.ell^2)/3*(Delta_cl_d(2)-3*P.b/(P.m*P.ell^2));


% transfer functions for plant and controller
Plant = tf([2/P.m/P.ell^2],[1, 2*P.b/P.m/P.ell^2, 0]);
C_pid = tf([(P.kd+P.kp*P.sigma),(P.kp+P.ki*P.sigma),P.ki],[P.sigma,1,0]);

loopshape_arm


