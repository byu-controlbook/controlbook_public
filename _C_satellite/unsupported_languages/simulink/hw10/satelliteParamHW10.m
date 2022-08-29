% satellite parameter file


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

% DC gain for outer loop
P.k_DC_phi = P.k*k_DC_th*P.kp_phi/(P.k+P.k*k_DC_th*P.kp_phi);


sprintf('DC_gain: %f\nkp_th: %f\nkd_th: %f\nkp_phi: %f\nkd_phi: %f\n',...
    k_DC_th, P.kp_th, P.kd_th, P.kp_phi, P.kd_phi)
