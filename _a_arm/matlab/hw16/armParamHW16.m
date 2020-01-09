% single link arm parameter file
addpath ./.. % adds the parent directory to the path
armParam % general parameters

% load parameters from HW 10
addpath ./../hw10
armParamHW10

% transfer function for robot arm
th_e = 0;
G = tf([2/P.m/P.ell^2],[1, 2*P.b/P.m/P.ell^2, -3*P.g*sin(th_e)/2/P.ell]);
figure(3), clf, bode(G), grid on

C_pid = tf([(P.kd+P.kp*P.sigma),(P.kp+P.ki*P.sigma),P.ki],[P.sigma,1,0])
figure(4), clf, bode(G), grid on
hold on
bode(series(G,C_pid))
legend('plant', 'PID control')

