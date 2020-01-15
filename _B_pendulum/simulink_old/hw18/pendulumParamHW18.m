% inverted Pendulum parameter file

% Compute inner and outer open-loop transfer functions
% P_in = tf([-2/P.m2/P.ell],[1,0,-2*(P.m1+P.m2)*P.g/P.m2/P.ell]);
% P_out = tf([P.g],[1,0,0]);

temp = P.m1*P.ell/6+P.m2*2*P.ell/3;
P_in = tf([-1/temp],[1,0,-(P.m1+P.m2)*P.g/temp]);
P_out = tf([-2*P.ell/3,0,P.g],[1,0,0]);

