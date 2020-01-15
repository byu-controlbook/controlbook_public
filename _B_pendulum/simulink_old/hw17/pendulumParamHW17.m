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

% margin and bode plots 
figure(1), clf, margin(P_in*C_in), grid on, hold on
bode(P_in*C_in/(1+P_in*C_in)) 
margin(P_out*C_out)
bode(P_out*C_out/(1+P_out*C_out))
legend('Open Loop-Inner', 'Closed Loop-Inner','Open Loop-Outer', 'Closed Loop-Outer')
