% inverted Pendulum parameter file

% dirty derivative parameters
P.sigma = 0.001; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       PD Control: Time Design Strategy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
tr_th   = 0.35;        % Rise time for inner loop (theta)
zeta_th = 0.707;       % Damping Coefficient for inner loop (theta)
M       = 20.0;        % Time scale separation between inner and outer loop
zeta_z  = 0.707;       % Damping Coefficient fop outer loop (z)

% saturation limits
P.F_max = 5;                   % Max Force, N
error_max = 1;        		   % Max step size,m
theta_max = 30.0*pi/180.0;  % Max theta, rads

%---------------------------------------------------
%                    Inner Loop
%---------------------------------------------------
% parameters of the open loop transfer function
b0_th = -2.0/(P.m2*P.ell);
a1_th = 0.0;
a0_th = -2.0*(P.m1+P.m2)*P.g/(P.m2*P.ell);
% Desired natural frequency
wn_th = 2.2/tr_th;     
% coefficients for desired inner loop
% Delta_des(s) = s^2 + alpha1*s + alpha0 = s^2 + 2*zeta*wn*s + wn^2
alpha1_th = 2.0*zeta_th*wn_th;
alpha0_th = wn_th^2;

% compute gains
% Delta(s) = s^2 + (a1 + b0*kd)*s + (a0 + b0*kp)
P.kp_th = (alpha0_th-a0_th)/b0_th;
P.kd_th = (alpha1_th-a1_th)/b0_th;

% P.kp_th = 0;      % Show that the pendulum falls without feedback

DC_gain = P.kp_th/((P.m1+P.m2)*P.g+P.kp_th);

%---------------------------------------------------
%                    Outer Loop
%---------------------------------------------------

% The outer loop cannot be stabilized with PD control
% (kd*s + kp) alone. Because of this, we will only 
% close the pendulum angle loop and let the cart 
% drift around as the pendulum is balanced. In upcoming
% chapters, we will look beyond PD/PID to close the outer
% loop.

sprintf('DC_gain: %f\nkp_th: %f\nkd_th: %f\n',...
    DC_gain, P.kp_th, P.kd_th)


