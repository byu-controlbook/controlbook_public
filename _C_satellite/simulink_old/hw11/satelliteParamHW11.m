% satellite parameter file

% tuning parameters
wn_th   = 0.6;
zeta_th = 0.707;
wn_phi    = 1.1;
zeta_phi  = 0.707;

% state space design
A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    -P.k/P.Js,P.k/P.Js,-P.b/P.Js,P.b/P.Js;...
    P.k/P.Jp,-P.k/P.Jp,P.b/P.Jp,-P.b/P.Jp;...

];
B = [0; 0; 1/P.Js; 0];
C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];

% gain selection
ol_char_poly = charpoly(A);
des_char_poly=conv(...
    [1,2*zeta_th*wn_th,wn_th^2],...
    [1,2*zeta_phi*wn_phi,wn_phi^2]);
des_poles = roots(des_char_poly);
% is the system controllable?
if rank(ctrb(A,B))~=4
    disp('System Not Controllable'); 
else
    P.K = place(A,B,des_poles);
    Cr = [0, 1, 0, 0];
    P.kr = -1/(Cr*inv(A-B*P.K)*B);
end

sprintf('K: (%f, %f, %f, %f)\nkr: %f\n', ...
    P.K(1), P.K(2), P.K(3), P.K(4), P.kr)
