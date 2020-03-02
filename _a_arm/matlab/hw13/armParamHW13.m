% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
armParam % general arm parameters

% tuning parameters
wn = 5.5;  % natural frequency for control
zeta = 0.707; % damping ratio for control
integrator_pole = -1;  % integrator pole for control
wn_obs = 10;  % natural frequency for observer
zeta_obs = 0.707; % damping ratio for observer

% equalibrium torque
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

% desired closed loop polynomial
% gains for pole locations
des_char_poly = conv([1,2*zeta*wn,wn^2],...
                     poly(integrator_pole));
des_poles = roots(des_char_poly);


% is the system controllable?
if rank(ctrb(A1,B1))~=3
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);
end

% observer design

% desired observer poles
des_obsv_char_poly = [1,2*zeta*wn_obs,wn_obs^2];
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A,P.C))~=2
    disp('System Not Observable'); 
else % if so, compute gains
    P.L = place(P.A',P.C',des_obsv_poles)'; 
end


fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)
fprintf('\t L^T: [%f, %f]\n', P.L(1), P.L(2))




