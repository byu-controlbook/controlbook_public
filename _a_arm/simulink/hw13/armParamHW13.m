% sample rate
P.Ts = 0.01;

% equilibrium torque
P.theta_e = 0*pi/180;
P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e);

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

% form augmented system
A1 = [P.A, zeros(2,1); -P.C, 0];
B1 = [P.B; 0];

% tuning parameters for control
wn = 5.5;  
zeta = 0.707;
integrator_pole = -1;

% compute gains
des_char_poly = conv(...
    [1,2*zeta*wn,wn^2],...
    poly(integrator_pole));
des_poles = roots(des_char_poly);
if rank(ctrb(A1,B1))~=3, 
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);
end

% observer design
% tuning parameters for observer
wn_obs = 10;
zeta_obs = 0.707;

% desired observer poles
des_obsv_char_poly = [1,2*zeta*wn,wn^2];
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A,P.C))~=2, 
    disp('System Not Observable'); 
else % if so, compute gains
    P.L = place(P.A',P.C',des_obsv_poles)'; 
end









