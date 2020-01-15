% inverted Pendulum parameter file

% saturation limits
P.F_max = 5;                   % Max Force, N

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% observer based control with integrator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters for control
tr_z = 1.5;            % rise time for position 
tr_theta = 0.5;        % rise time for angle
zeta_z   = 0.707;      % damping ratio position
zeta_th  = 0.707;      % damping ratio angle
integrator_pole = -10; % integrator pole
% tuning parameters for observer
tr_z_obs = tr_z/10;
tr_theta_obs = tr_theta/10;
% pole for disturbance observer
dist_obsv_pole = -1;   


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
wn_th = 2.2/tr_theta; % natural frequency for angle
wn_z = 2.2/tr_z; % natural frequency for position
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

% observer design
% form augmented system for disturbance observer
A2 = [P.A, P.B; zeros(1,4), zeros(1,1)];
C2 = [P.C, zeros(2,1)];
% pick observer poles
% natural frequency - position
wn_z_obs = 2.2/tr_z_obs; 
% natural frequency - angle
wn_th_obs = 2.2/tr_theta_obs; 

des_obsv_char_poly = conv(...
    [1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
    [1,2*zeta_th*wn_th_obs,wn_th_obs^2]);
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(A2,C2))~=5, 
    disp('System Not Observable'); 
else % if so, compute gains
    L2 = place(A2', C2',...
        [des_obsv_poles;dist_obsv_pole])';
    P.L = L2(1:4,:);
    P.Ld = L2(5,:);
end

sprintf('K:\t[%f, %f, %f, %f]\nki:\t%f\nL^T:...
	\t[%f, %f, %f, %f;\n\t %f, %f, %f, %f]\nLd:\t[%f, %f]',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.ki,...
    P.L(1,1), P.L(2,1), P.L(3,1), P.L(4,1),...
    P.L(1,2), P.L(2,2), P.L(3,2), P.L(4,2),...
    P.Ld(1), P.Ld(2))







