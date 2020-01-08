3% sample rate
P.Ts = 0.01;

% equilibrium torque
P.theta_e = 0*pi/180;
P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e);

% saturation constraint
P.tau_max = 1;
tau_max = P.tau_max-P.tau_e;

%---------------------
% state space design 
P.A = [...
    0, 1;...
    0, -3*P.b/P.m/(P.ell^2);...
    ];
P.B = [0; 3/P.m/(P.ell^2) ];
P.C = [...
    1, 0;...
    ];
% form augmented system for integrator
A1 = [P.A, zeros(2,1); -P.C, 0];
B1 = [P.B; 0];

%  tuning parameters for control
tr = 0.2;  
zeta = 0.707;
integrator_pole = -1.0;

% desired closed loop polynomial
wn = 2.2/tr;
des_char_poly = conv(...
    [1,2*zeta*wn,wn^2],...
    poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=3, 
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);
end

% observer design
% form augmented system for disturbance observer
A2 = [P.A, P.B; zeros(1,2), 0];
C2 = [P.C, 0];

% observer design
% tuning parameters for observer
wn_obs = 10;
zeta_obs = 1.707;
dist_pole = -5.5;

% desired observer poles
des_obsv_char_poly = conv(...
    [1,2*zeta*wn,wn^2],...
    poly(dist_pole));
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(A2,C2))~=3 
    disp('System Not Observable'); 
else % if so, compute gains
    L2 = place(A2',C2',des_obsv_poles)'; 
    P.L = L2(1:2);
    P.Ld = L2(3);
end









