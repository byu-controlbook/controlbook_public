% satellite parameter file
addpath ./.. % adds the parent directory to the path
satelliteParam % general satellite parameters

% load parameters from HW 10
addpath ./../hw10
satelliteParamHW10

% transfer functions
% P_in = tf([1/P.Js],[1,P.b/P.Js,P.k/P.Js]);
P_in = tf([1],[P.Js+P.Jp,0,0]);
P_out = tf([P.b/P.Jp, P.k/P.Jp],[1,P.b/P.Jp,P.k/P.Jp]);

C_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
C_out = tf([(P.kd_phi+P.kp_phi*P.sigma),(P.kp_phi+P.ki_phi*P.sigma),P.ki_phi],[P.sigma,1,0]);

hand = figure(2), clf, 
bode(P_in), grid on
hold on
bode(series(C_in,P_in))

title('Satellite, Inner Loop')

% Use this to add in the 1/s^2 line to create the book figure
% children = get(hand,'Children');
% magChild = children(3);
% axes(magChild)
% w = logspace(-5, 0);
% plot(w,-40*log10(w),'--k')

legend('No control', 'PD')

figure(3), clf, 
bode(P_out), grid on
hold on
bode(series(C_out,P_out))
legend('No control', 'PID')
title('Satellite, Outer Loop')




