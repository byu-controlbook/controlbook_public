% satellite parameter file


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state feedback control with integrator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
wn_th   = 0.6;
zeta_th = 0.707;
wn_phi    = 1.1;
zeta_phi  = 0.707;
integrator_pole = -1;

% state space design
A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    -P.k/P.Js, P.k/P.Js, -P.b/P.Js, P.b/P.Js;...
    P.k/P.Jp, -P.k/P.Jp, P.b/P.Jp, -P.b/P.Jp;...

];
B = [0; 0; 1/P.Js; 0];
C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];

% form augmented system
Cout = [0,1,0,0];
A1 = [A, zeros(4,1); -Cout, 0];
B1 = [B; 0];

% compute gains
ol_char_poly = charpoly(A);
des_char_poly = conv(...
    conv([1,2*zeta_th*wn_th,wn_th^2],...
         [1,2*zeta_phi*wn_phi,wn_phi^2]),...
    poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=5, 
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:4);
    P.ki = K1(5);
end

sprintf('K: [%f, %f, %f, %f]\nki: %f\n',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.ki)







