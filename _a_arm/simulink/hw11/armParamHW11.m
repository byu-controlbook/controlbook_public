% sample rate
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); 

% equalibrium torque
P.theta_e = 0*pi/180;
P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e);

%---------------------
% state space design 
A = [...
    0, 1;...
    0, -3*P.b/P.m/(P.ell^2);...
    ];
B = [0; 3/P.m/(P.ell^2) ];
C = [...
    1, 0;...
    ];

%  tuning parameters
%tr = 0.8; % part (a)
% tuned to get fastest possible rise time 
%  before saturation.
tr = 0.4;  
zeta = 0.707;

% desired closed loop polynomial
wn = 2.2/tr;
% gains for pole locations
des_char_poly = [1,2*zeta*wn,wn^2];
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A,B))~=2, 
    disp('System Not Controllable'); 
else
    P.K = place(A,B,des_poles); 
    P.kr = -1/(C*inv(A-B*P.K)*B);
end







