% inverted Pendulum parameter file
addpath ./.. % adds the parent directory to the path
pendulumParam % general pendulum parameters

% saturation limits
P.F_max = 5;                   % Max Force, N

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% observer based control with integrator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters for control
tr_z = 1.5;            % rise time for position 
tr_theta = 0.5;        % rise time for angle
zeta_z   = 0.707;      % damping ratio position
zeta_th  = 0.707;      % damping ratio angle
integrator_pole = -3; % integrator pole


% state space equations
P.A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -3*P.m1*P.g/4/(.25*P.m1+P.m2), -P.b/(.25*P.m1+P.m2), 0;...
    0, 3*(P.m1+P.m2)*P.g/2/(.25*P.m1+P.m2)/P.ell, ...
                              3*P.b/2/(.25*P.m1+P.m2)/P.ell, 0;...
];
P.B = [0; 0; 1/(.25*P.m1+P.m2); -3/2/(.25*P.m1+P.m2)/P.ell ];
P.C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];
% form augmented system
Cout = [1,0,0,0];
A1 = [P.A, zeros(4,1); -Cout, 0];
B1 = [P.B; 0];

% compute control gains
wn_th = 2.2/tr_theta; % natural frequency - angle
wn_z = 2.2/tr_z; % natural frequency - position

% tuning parameters for observer
wn_th_obs   = 5*wn_th;
wn_z_obs    = 5*wn_z;

des_char_poly = conv(...
    conv([1,2*zeta_z*wn_z,wn_z^2],...
         [1,2*zeta_th*wn_th,wn_th^2]),...
    poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=5
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:4);
    P.ki = K1(5);
end

% computer observer gains
des_obsv_char_poly = conv(...
    [1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
    [1,2*zeta_th*wn_th_obs,wn_th_obs^2]);
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A,P.C))~=4
    disp('System Not Observable'); 
else % if so, compute gains
    P.L = place(P.A', P.C', des_obsv_poles)';
end

sprintf('K: [%f, %f, %f, %f]\nki: %f\nL: [%f, %f, %f, %f]^T',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.ki,...
        P.L(1), P.L(2), P.L(3), P.L(4))







