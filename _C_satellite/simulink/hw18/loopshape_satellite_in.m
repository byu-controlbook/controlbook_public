addpath('./..')
satelliteParam

% load parameters from HW 10
addpath ./../hw10
satelliteParamHW10

% transfer functions
P_in = tf([P.sigma, 1],[P.sigma*P.Js, (P.sigma*P.b+P.Js),... 
                       (P.b+P.kd_th+P.sigma*P.k),P.k]);
Plant = P_in;
figure(2), clf
    handle=bode(Plant,logspace(-4,4));
    hold on
    grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %--- general tracking specification ---
        omega_r = 10^-2;  % track signals below this frequency
        gamma_r = 10^(-40/20);  % tracking error below this value
        w = logspace(log10(omega_r)-2,log10(omega_r));
        plot(w,20*log10(1/gamma_r)*ones(size(w)),'g')
        
%         %--- output disturbance specification ---
%         omega_dout = 10^-3;  % reject output disturbances below this frequency
%         gamma_dout = 0.001;  % amountn of output disturbance in output
%         w = logspace(log10(omega_dout)-2,log10(omega_dout));
%         plot(w,20*log10(1/gamma_dout)*ones(size(w)),'g')

%         %--- input disturbance specification ---
%         omega_din = 10^-1;  % reject input disturbances below this frequency
%         gamma_din = 0.1;  % amountn of input disturbance in output
%         w = logspace(log10(omega_din)-2,log10(omega_din));
%         Pmag=bode(P,w);
%         for i=1:size(Pmag,3), Pmag_(i)=Pmag(1,1,i); end
%         plot(w,20*log10(1/gamma_din)*ones(1,length(Pmag_))+20*log10(Pmag_),'g')

        %--- noise specification ---
        omega_n = 2*10^1;  % attenuate noise above this frequency
        gamma_n = 10^(-40/20);   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,20*log10(gamma_n)*ones(size(w)),'g')
        
%         %--- steady state tracking of step ---
%         gamma_0 = .01;
%         w = logspace(-5, 0);
%         plot(w,20*log10(1/gamma_0 -1)*ones(size(w)),'g')
        
%         %--- steady state tracking of ramp ---
%         gamma_1 = .03;
%         w = logspace(-4, 0);
%         plot(w,20*log10(1/gamma_1)-20*log10(w),'g')

%         %--- steady state tracking of parabola ---
%         gamma_2 = .01;
%         w = logspace(-5, 0);
%         plot(w,20*log10(1/gamma_2)-40*log10(w),'g')

   figure(2), bode(Plant,logspace(-4,4)), grid on
   %print('../../../figures/hw_satellite_compensator_in_design_1','-dpdf','-bestfit')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% proportional control: change cross over frequency
     kp = 30;
     C = C*kp;
     figure(2), margin(Plant*C), grid on
%print('../../../figures/hw_satellite_compensator_in_design_2','-dpdf','-bestfit')
     
% % phase lag: add gain at low frequency (tracking, dist rejection)
%      % low frequency gain = K*z/p
%      % high frequency gain = K
%      z = 1; % frequency at which lag ends
%      M = 3;  % separation between pole and zero
%      Lag = tf([1,z],[1,z/M]);
%      C = C*Lag;
%      figure(2), margin(Plant*C),  % update plot

% % integral control: increase steady state tracking and dist rejection
%      k_I = 1.5; % frequency at which integral action ends
%      Integrator = tf([1,k_I],[1,0]);
%      C = C*Integrator;
%      figure(2), margin(Plant*C),  % update plot

% % phase lead: increase PM (stability)
%     wmax = 0.1; % location of maximum frequency bump
%     M    = 5; % separation between zero and pole
%     Lead =tf(M*[1,wmax/sqrt(M)],[1,wmax*sqrt(M)]);
%     C = C*Lead;
%     figure(2), margin(Plant*C),  % update plot

% low pass filter: decrease gain at high frequency (noise)
     p = 10;
     LPF = tf(p,[1,p]);
     C = C*LPF;
     figure(2), margin(Plant*C)  % update plot
%print('../../../figures/hw_satellite_compensator_in_design_3','-dpdf','-bestfit')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
  F = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % prefilter: reduce closed loop peaking at cross over
%     % notch filter
%     wnotch = 40;  % center frequency of notch
%     D = 5; % depth of notch
%     M  = 20;  % width of the notch
%     NOTCH = tf([1,(D/sqrt(M)+sqrt(M)/D)*wnotch,wnotch^2],...
%                [1,(1/sqrt(M)+sqrt(M))*wnotch,wnotch^2]);
%     F = F*NOTCH;
%     % low pass filter
%     p = 9;  % frequency to start the LPF
%     LPF = tf(p, [1,p]);
%     F = F*LPF;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create plots for analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Open-loop tranfer function 
  OPEN = Plant*C;
% closed loop transfer function from R to Y
  CLOSED_R_to_Y = minreal((Plant*C/(1+Plant*C)));
% closed loop transfer function from R to U
  CLOSED_R_to_U = minreal((C/(1+C*Plant)));

% update figure 2
figure(2), margin(Plant*C), 
grid on
    
figure(3), clf
    subplot(3,1,1), 
        bodemag(CLOSED_R_to_Y), hold on
        bodemag(CLOSED_R_to_Y*F)
        title('Closed Loop Bode Plot'), grid on
    subplot(3,1,2), 
        step(CLOSED_R_to_Y), hold on
        step(CLOSED_R_to_Y*F)
        title('Closed loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U), hold on
        step(CLOSED_R_to_U*F)
        title('Control effort for step response'), grid on
%print('../../../figures/hw_satellite_compensator_in_design_4','-dpdf','-bestfit')
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[num,den] = tfdata(C,'v');
[P.Ain_C,P.Bin_C,P.Cin_C,P.Din_C]=tf2ss(num,den);

%[num,den] = tfdata(F,'v');
%[P.A_F, P.B_F, P.C_F, P.D_F] = tf2ss(num,den);

C_in = C;
