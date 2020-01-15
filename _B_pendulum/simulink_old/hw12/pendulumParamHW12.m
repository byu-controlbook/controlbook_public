% inverted Pendulum parameter file

% dirty derivative parameters
P.sigma = 0.05; 
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); 

% saturation limits
P.F_max = 5;                   % Max Force, N
theta_max = 30.0*pi/180.0;  % Max theta, rads

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state feedback control with integrator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
tr_z = 1.5;            % rise time for position 
tr_theta = 0.5;        % rise time for angle
zeta_z   = 0.707;      % damping ratio position
zeta_th  = 0.707;      % damping ratio angle
integrator_pole = -2; % integrator pole

% state space equations
A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -3*P.m1*P.g/4/(.25*P.m1+P.m2), -P.b/(.25*P.m1+P.m2), 0;...
    0, 3*(P.m1+P.m2)*P.g/2/(.25*P.m1+P.m2)/P.ell, ...
                              3*P.b/2/(.25*P.m1+P.m2)/P.ell, 0;...
];
B = [0; 0; 1/(.25*P.m1+P.m2); -3/2/(.25*P.m1+P.m2)/P.ell ];
C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];
% form augmented system
Cout = [1,0,0,0];
A1 = [A, zeros(4,1); -Cout, 0];
B1 = [B; 0];


% compute gains
wn_th = 2.2/tr_theta; % natural frequency - angle
wn_z = 2.2/tr_z; % natural frequency - position
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

sprintf('K: [%f, %f, %f, %f]\nki: %f\n',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.ki)







