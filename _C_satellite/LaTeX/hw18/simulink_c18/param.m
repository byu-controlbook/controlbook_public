%clear all

% initial conditions
P.theta0    = 0;
P.phi0      = 0;
P.thetadot0 = 0;
P.phidot0   = 0;

% parameters known to controller
P.Js = 5; % kg m^2
P.Jp = 1;  % kg m^2
P.k = 0.15; % N m 
P.b = 0.05; % N m s

% maximum torque
P.taumax = 5; %Nm

% sample rate of controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;

% tunning parameters
%tr_phi = 20;  % rise time for outer loop 
tr_phi = 7;
zeta_phi = 0.707; % damping ratio for outer loop
M = 10;    % time scale separation between inner and outer loop
zeta_th  = 0.707; % damping ratio for inner loop
P.ki_phi = 0.0; % integral gain for outer loop

% PD design for inner loop
tr_th    = tr_phi/M;
wn_th    = 2.2/tr_th;
P.kp_th  = wn_th^2*P.Js-P.k;
P.kd_th  = 2*zeta_th*wn_th*P.Js-P.b;

% DC gain for inner loop
k_DC_th = P.kp_th/(P.k+P.kp_th);

%PD design for outer loop
zeta_phi = 0.707;
wn_phi = 2.2/tr_phi;
tmp = inv([P.k*k_DC_th, -P.Jp*P.b*k_DC_th; P.b*k_DC_th, P.k*k_DC_th-2*zeta_phi*wn_phi*P.b*k_DC_th])*[-P.k+P.Jp*wn_phi^2; -P.b+2*P.Jp*zeta_phi*wn_phi];
P.kp_phi = tmp(1);
P.kd_phi = tmp(2);

% transfer functions
%P_in = tf([1/P.Js],[1,P.b/P.Js,P.k/P.Js]);
%P_in = tf([1/P.Js],[1,(P.b+P.kd_th)/P.Js,P.k/P.Js]);
P_in = tf([P.sigma, 1],[P.sigma*P.Js, (P.sigma*P.b+P.Js),... 
                       (P.b+P.kd_th+P.sigma*P.k),P.k]);
%P_out = tf([P.b/P.Jp, P.k/P.Jp],[1,P.b/P.Jp,P.k/P.Jp]);
%P_out = tf([P.b, P.k],[(P.Jp+P.b*P.kd_phi),(P.b+P.k*P.kd_phi),P.k]);
P_out = tf([P.sigma*P.b, P.b+P.sigma*P.k, P.k],...
           [P.sigma*P.Jp, P.Jp+P.b*P.kd_phi+P.sigma*P.b,...
            P.b+P.k*P.kd_phi+P.sigma*P.k, P.k]);




