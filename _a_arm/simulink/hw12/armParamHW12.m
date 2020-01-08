% sample rate
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); 

% equilibrium torque
P.theta_e = 0*pi/180;
P.tau_e   = P.m*P.g*P.ell/2*cos(P.theta_e);

% saturation constraint
P.tau_max = 1;
tau_max = P.tau_max-P.tau_e;

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
% form augmented system
A1 = [A, zeros(2,1); -C, 0];
B1 = [B; 0];

%  tuning parameters
tr = 0.4;  
zeta = 0.707;
integrator_pole = -5;

% desired closed loop polynomial
wn = 2.2/tr;
% gains for pole locations
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








