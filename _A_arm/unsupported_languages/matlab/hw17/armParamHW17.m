% single link arm parameter file
addpath ./.. % adds the parent directory to the path
armParam % general parameters

% load parameters from HW 10
addpath ./../hw10
armParamHW10

% transfer functions for plant and controller
Plant = tf([2/P.m/P.ell^2],[1, 2*P.b/P.m/P.ell^2, 0]);
C_pid = tf([(P.kd+P.kp*P.sigma),(P.kp+P.ki*P.sigma),P.ki],[P.sigma,1,0]);

% margin and bode plots for PID control
figure(3), clf, margin(Plant*C_pid), grid on, hold on
bode(Plant*C_pid/(1+Plant*C_pid)) 
legend('Open Loop', 'Closed Loop')

figure(4), clf, grid on, hold on
step(Plant*C_pid/(1+Plant*C_pid))
step(C_pid/(1+Plant*C_pid))
legend('output y', 'input u')

