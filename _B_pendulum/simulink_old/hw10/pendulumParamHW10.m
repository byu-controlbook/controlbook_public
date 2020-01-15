% inverted Pendulum parameter file

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       PD Control: Time Design Strategy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
tr_z = 2;%5;        % rise time for outer loop   - Can't really push saturation limits and remain stable.  
zeta_z   = 0.707;   % damping ratio for outer loop
M = 8;%10;          % time scale separation between inner and outer loop
zeta_th  = 0.707;   % damping ratio for inner loop
P.ki_z = 0.0001;    % select integrator gain


% saturation limits
P.F_max = 5;                   % Max Force, N
error_max = 1;        		   % Max step size,m
theta_max = 30.0*pi/180.0;  % Max theta, rads

%---------------------------------------------------
%                    Inner Loop
%---------------------------------------------------
% gains for inner loop
tr_theta = tr_z/M;  % rise time for inner loop
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
P.kp_th  = -(P.m1+P.m2)*P.g - P.m2*P.ell/2*wn_th^2; % kp - inner loop
P.kd_th  = -zeta_th*wn_th*P.m2*P.ell; % kd - inner loop
% DC gain for inner loop
k_DC_gain = P.kp_th/((P.m1+P.m2)*P.g+P.kp_th);

%---------------------------------------------------
%                    Outer Loop
%---------------------------------------------------

%PD design for outer loop
wn_z     = 2.2/tr_z; % natural frequency for outer loop
P.kp_z = wn_z^2/P.g/k_DC_gain;
P.kd_z = 2*zeta_z*wn_z/P.g/k_DC_gain;


sprintf('DC_gain: %f\nkp_th: %f\nkd_th: %f\nkp_z: %f\nkd_z: %f\nki_z: %f\n',...
    k_DC_gain, P.kp_th, P.kd_th, P.kp_z, P.kd_z, P.ki_z)

