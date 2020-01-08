addpath('../.')
arm_param


% transfer function for robot arm
th_e = 0;
G = tf([2/P.m/P.ell^2],[1, 2*P.b/P.m/P.ell^2, -3*P.g*sin(th_e)/2/P.ell]);
figure(1), clf, bode(G), grid on
