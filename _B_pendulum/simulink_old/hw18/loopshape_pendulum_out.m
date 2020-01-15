% param
% loopshape_pendulum_in

Plant = minreal(P_out*(P_in*C_in/(1+P_in*C_in)));
figure(2), clf
    hold on
    grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %--- general tracking specification ---
        omega_r = 0.0032;  % track signals below this frequency
        gamma_r = 0.01;  % tracking error below this value
        w = logspace(log10(omega_r)-2,log10(omega_r));
        plot(w,(1/gamma_r)*ones(size(w)),'g')
        
        %--- noise specification ---
        omega_n = 1000;  % attenuate noise above this frequency
        gamma_n = 0.01;   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,gamma_n*ones(size(w)),'g')
        
figure(2), bode(Plant,logspace(-3,5)), grid on
%print('../../../figures/hw_pendulum_compensator_out_design_1','-depsc')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = tf([1],[1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% phase lead: increase PM (stability)
% At desired crossover frequency, PM = -3
% Add 70 deg of PM with lead
    w_max = 1.1; % location of maximum frequency bump (desired crossover)
    phi_max = 70*pi/180;
    M = (1+sin(phi_max))/(1-sin(phi_max)) % lead ratio
    z = w_max/sqrt(M)
    p = w_max*sqrt(M)
    Lead = tf([1/z 1],[1/p 1]);
    C = C*Lead;
    figure(2), bode(Plant*C)  % update plot 

% find gain to set crossover at w_max = 1.1 rad/s
    [m,p] = bode(Plant*C,1.1);
    K = 1/m;
    C = K*C;
    figure(2), margin(Plant*C), grid on,  % update plot
    legend('plant alone','lead, K=1','lead, K=0.0154');
% % % % print('../../../figures/hw_pendulum_compensator_out_design_2','-depsc')
%     figure(2), bode(Plant*C,logspace(-3,5)), grid on,  % update plot
% 
% % Tracking constraint not satisfied -- add lag compensation to boost low
% % frequency gain

% Find gain increase needed at omega_r
    [m,p] = bode(Plant*C,omega_r);
    gain_increase_needed = 1/gamma_r/m
    % Minimum gain increase at low frequencies is 4.8. Let lag ratio be 8.
    M = 8;
    p = omega_r;    % Set pole at omega_r
    z = M*p;        % Set zero at M*omega_r
    Lag = tf(M*[1/z 1],[1/p 1]);
    C = C*Lag;
    figure(2), margin(Plant*C)
    legend('lead','lead+lag');
    % print('../../../figures/hw_pendulum_compensator_out_design_3','-depsc')
    figure(2), bode(Plant*C)

    % Noise attenuation constraint not quite satisfied
    % Can be satisfied by reducing gain at 400 rad/s by a factor of 2
    % Use a low-pass filter
        m = 0.5;    % attenuation factor
        a = m*omega_n*sqrt(1/(1-m^2));
        lpf = tf(a,[1 a]);
        C = lpf*C;
        figure(2), margin(Plant*C),  % update plot
        legend('lead','lead+lag','lead+lag+lowpass');
% % % print('../../../figures/hw_pendulum_compensator_out_design_4','-depsc')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
  F = tf([1],[1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% low pass filter
    p = 2;  % frequency to start the LPF
    LPF = tf(p,[1 p]);
    F = F*LPF

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create plots for analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Open-loop tranfer function 
  OPEN = Plant*C;
% closed loop transfer function from R to Y
  CLOSED_R_to_Y = (Plant*C/(1+Plant*C));
% closed loop transfer function from R to U
  CLOSED_R_to_U = (C/(1+C*Plant));

figure(3), clf
    subplot(3,1,1), 
        bodemag(CLOSED_R_to_Y), hold on
        bodemag(CLOSED_R_to_Y*F)
        bodemag(P_in*C_in/(1+P_in*C_in))
        title('Closed-loop Bode plot'), grid on
        legend('closed loop','closed loop + prefilter','open loop');
    subplot(3,1,2), 
        step(CLOSED_R_to_Y), hold on
        step(CLOSED_R_to_Y*F)
        title('Closed-loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U), hold on
        step(CLOSED_R_to_U*F)
        title('Control effort for step response'), grid on
% print('../../../figures/hw_pendulum_compensator_out_design_5','-depsc')


        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C=minreal(C);
[num,den] = tfdata(C,'v');
[P.Aout_C,P.Bout_C,P.Cout_C,P.Dout_C]=tf2ss(num,den);

[num,den] = tfdata(F,'v');
[P.Aout_F, P.Bout_F, P.Cout_F, P.Dout_F] = tf2ss(num,den);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to discrete transfer functions for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C_out_d = c2d(C,P.Ts,'tustin')
[P.Cout_d_num,P.Cout_d_den] = tfdata(C_out_d,'v');

F_d = c2d(F,P.Ts,'tustin');
[P.F_d_num,P.F_d_den] = tfdata(F_d,'v');

C_out = C;


