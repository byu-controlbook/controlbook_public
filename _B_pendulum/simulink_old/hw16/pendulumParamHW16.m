% inverted Pendulum parameter file
addpath ./.. % adds the parent directory to the path
pendulumParam % general pendulum parameters

% load parameters from HW 10
addpath ./../hw10
pendulumParamHW10

% Compute inner and outer open-loop transfer functions
temp = P.m1*P.ell/6+P.m2*2*P.ell/3;
P_in = tf([-1/temp],[1,0,-(P.m1+P.m2)*P.g/temp]);
P_out = tf([-2*P.ell/3,0,P.g],[1,0,0]);

% Compute inner and outer closed-loop transfer functions
C_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
C_out = tf([(P.kd_z+P.kp_z*P.sigma),(P.kp_z+P.ki_z*P.sigma),P.ki_z],...
                                                         [P.sigma,1,0]);


% display bode plots of transfer functions
figure(1), clf, 
bode(P_in), grid on
hold on
bode(series(C_in,P_in))
legend('No control', 'PD')
title('Inverted Pendulum, Inner Loop')

figure(2), clf, 
bode(P_out), grid on
hold on
bode(series(C_out,P_out))
bode([1],[1,0])
legend('No control', 'PID','1/s')
title('Inverted Pendulum, Outer Loop')





