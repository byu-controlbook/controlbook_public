% inverted Pendulum parameter file

% dirty derivative parameters
P.sigma = 0.01; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       PD Control: Time Design Strategy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
tr_th   = 0.5;       % Rise time for inner loop (theta)
zeta_th = 0.7;       % Damping Coefficient for inner loop (theta)
M       = 10.0;      % Time scale separation between inner and outer loop
zeta_z  = 0.7;       % Damping Coefficient fop outer loop (z)

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
DC_gain = P.kp_th/((P.m1+P.m2)*P.g+P.kp_th);

%---------------------------------------------------
%                    Outer Loop
%---------------------------------------------------
tr_z = tr_th*M;
wn_z = 2.2/tr_z; % natural frequency for outer loop

bth = 2/(P.m2*P.ell);
ath = (P.m1+P.m2)*P.g*bth;

% Inner-loop CLTF
th_cl = tf(-bth*P.kp_th,[1 -bth*P.kd_th -(ath+bth*P.kp_th)])

a = 0.33;
C = tf([1/a 1],[1/(12*a) 1]);
[numC,denC] = tfdata(C,'v');

% C = 1;

% G is TF from Theta_des to Z
G = th_cl*tf(-[P.ell/2 0 -P.g],[1 0 0]);

figure(3); clf;
rlocus(C*G); hold on;
sgrid(zeta_z,[wn_z 100]);
% K = 0.02;
K = 0.0075;
rlocus(C*G,K,'r^'); 
hold off;
axis([-10 10 -10 10]);



