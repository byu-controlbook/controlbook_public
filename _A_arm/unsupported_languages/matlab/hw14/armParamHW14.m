% single link arm - parameter file
addpath ./.. % adds the parent directory to the path
armParam % general arm parameters

% tuning parameters
tr = 0.2;  % rise time for controller
zeta = 0.707;  % damping ratio for controller
integrator_pole = -1.0;
% tuning parameters for observer
wn_obs = 10;  % natural frequency for observer
zeta_obs = 1.707;  % damping ratio for observer
dist_pole = -5.5;  % disturbance observer pole

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
P.B1 = [P.B; 0];

% desired closed loop polynomial
wn = 2.2/tr;
des_char_poly = conv([1,2*zeta*wn,wn^2],...
                     poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1, P.B1))~=3
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1, P.B1, des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);
end

% des_char_poly = [1,2*zeta*wn,wn^2];
% des_poles = roots(des_char_poly);
% 
% if rank(ctrb(P.A,P.B))~=2 
%     disp('System Not Controllable'); 
% else
%     P.K = place(P.A,P.B,des_poles); 
%     P.kr = -1/(P.C*inv(P.A-P.B*P.K)*P.B);
% end
% P.ki =0;


% observer design
% form augmented system for disturbance observer
P.A2 = [P.A, P.B; zeros(1,2), 0];
P.C2 = [P.C, 0];

% desired observer poles
des_obsv_char_poly = conv([1,2*zeta*wn,wn^2],...
                          poly(dist_pole));
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A2, P.C2))~=3 
    disp('System Not Observable'); 
else % if so, compute gains
    P.L2 = place(P.A2', P.C2', des_obsv_poles)'; 
end

fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)
fprintf('\t L^T: [%f, %f, %f]\n', P.L2(1), P.L2(2), P.L2(3))




