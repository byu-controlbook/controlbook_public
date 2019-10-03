% satellite parameter file
addpath ./.. % adds the parent directory to the path
satelliteParam % general satellite parameters

% load parameters from HW 10
addpath ./../hw10
satelliteParamHW10

% transfer functions
P_in = tf([1/P.Js],[1,P.b/P.Js,P.k/P.Js]);
P_out = tf([P.b/P.Jp, P.k/P.Jp],[1,P.b/P.Jp,P.k/P.Jp]);

C_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
C_out = tf([(P.kd_phi+P.kp_phi*P.sigma),(P.kp_phi+P.ki_phi*P.sigma),P.ki_phi],[P.sigma,1,0]);

% margin and bode plots 
figure(2), clf, margin(P_in*C_in), grid on, hold on
bode(P_in*C_in/(1+P_in*C_in)) 
margin(P_out*C_out)
bode(P_out*C_out/(1+P_out*C_out))
legend('Open Loop-Inner', 'Closed Loop-Inner','Open Loop-Outer', 'Closed Loop-Outer')




