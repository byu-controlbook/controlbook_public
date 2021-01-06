% inverted pendulum - parameter file for hw11
addpath ./.. % adds the parent directory to the path
pendulumParam % general pendulum parameters

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      State Space Pole Placement
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
tr_theta = 0.5; % rise time for angle
M = 3.0;              % Time scale separation between inner and outer loop
tr_z = M*tr_theta; % 1.5;  % rise time for position 

zeta_z   = 0.707; % damping ratio position
zeta_th  = 0.707; % damping ratio angle

% state space equations
A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -3*P.m1*P.g/4/(.25*P.m1+P.m2), -P.b/(.25*P.m1+P.m2), 0;...
    0, 3*(P.m1+P.m2)*P.g/2/(.25*P.m1+P.m2)/P.ell,... 
        3*P.b/2/(.25*P.m1+P.m2)/P.ell, 0;...
];
B = [0; 0; 1/(.25*P.m1+P.m2); -3/2/(.25*P.m1+P.m2)/P.ell ];
C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];


% gain calculation
wn_th = 2.2/tr_theta;  % natural frequency for angle
wn_z = 2.2/tr_z;  % natural frequency for position
des_char_poly = conv([1, 2*zeta_z*wn_z, wn_z^2],...
                     [1, 2*zeta_th*wn_th, wn_th^2]);
des_poles = roots(des_char_poly);

% Compute the gains if the system is controllable
if rank(ctrb(A, B)) ~= 4
    disp('The system is not controllable')
else
    P.K = place(A, B, des_poles);
    P.kr = -1.0/(C(1,:)*inv(A-B*P.K)*B);
end

sprintf('K: (%f, %f, %f, %f)\nkr: %f\n',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.kr)







