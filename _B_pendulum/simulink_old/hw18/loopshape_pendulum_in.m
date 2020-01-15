addpath('./..')
pendulum_param
pendulumParamHW18

Plant = P_in;

figure(2), clf
    hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %--- noise specification ---
        omega_n = 200;   % attenuate noise above this frequency
        gamma_n = 0.1;   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,gamma_n*ones(size(w)),'g')
        

  figure(2), bode(Plant,logspace(-3,5)), grid on
% %print('../../../figures/hw_pendulum_compensator_in_design_1','-depsc')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% proportional control: correct for negative sign in plant
     K = -1;
     C = C*K;
     figure(2), bode(Plant*C)
     legend('K=1','K=-1')
% %      print('../../../figures/hw_pendulum_compensator_in_design_2','-depsc')

%      pause
     
% phase lead: increase PM (stability)
    w_max = 40; % location of maximum frequency bump (desired crossover)
    phi_max = 60*pi/180;
    M = (1+sin(phi_max))/(1-sin(phi_max)); % lead ratio
    z = w_max/sqrt(M)
    p = w_max*sqrt(M)
    Lead = tf([1/z 1],[1/p 1]);
    C = C*Lead;
    figure(2), bode(Plant*C), hold on, grid % update plot
    
% find gain to set crossover at w_max = 25 rad/s
    [m,p] = bode(Plant*C,25);
    K = 1/m;
    C = K*C;
    figure(2), margin(Plant*C)  % update plot
    legend('K=1','K=-1','lead, K=-1','lead, K=-45.1')
% %print('../../../figures/hw_pendulum_compensator_in_design_3','-depsc')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create plots for analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Open-loop tranfer function 
  OPEN = Plant*C;
% closed loop transfer function from R to Y
  CLOSED_R_to_Y = minreal((Plant*C/(1+Plant*C)));
% closed loop transfer function from R to U
  CLOSED_R_to_U = minreal((C/(1+C*Plant)));

figure(3), clf
    subplot(3,1,1), 
        bodemag(CLOSED_R_to_Y)
        title('Closed-loop Bode plot'), grid on
    subplot(3,1,2), 
        step(CLOSED_R_to_Y)
        title('Closed-loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U)
        title('Control effort for step response'), grid on

% % print('../../../figures/hw_pendulum_compensator_in_design_4','-depsc')        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[num,den] = tfdata(C,'v');
[P.Ain_C,P.Bin_C,P.Cin_C,P.Din_C]=tf2ss(num,den);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to discrete transfer functions for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C_in_d = c2d(C,P.Ts,'tustin')
[P.Cin_d_num,P.Cin_d_den] = tfdata(C_in_d,'v');

C_in = C;


