clear all

% Plant transfer function
Plant = tf(1,[1,1]);

figure(1), clf
    bode(Plant)
    hold on
    grid on
 figure(2), clf
    bodemag(Plant)
    hold on
    grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%         %--- general tracking specification ---
%         omega_r = 10^-3;  % track signals below this frequency
%         gamma_r = 0.001;  % tracking error below this value
%         w = logspace(log10(omega_r)-2,log10(omega_r));
%         plot(w,20*log10(1/gamma_r)*ones(size(w)),'g')
        
%         %--- output disturbance specification ---
%         omega_dout = 10^-3;  % reject output disturbances below this frequency
%         gamma_dout = 0.001;  % amountn of output disturbance in output
%         w = logspace(log10(omega_dout)-2,log10(omega_dout));
%         plot(w,20*log10(1/gamma_dout)*ones(size(w)),'g')

        %--- input disturbance specification ---
        omega_din = 10^-1;  % reject input disturbances below this frequency
        gamma_din = 0.1;  % amountn of input disturbance in output
        w = logspace(log10(omega_din)-2,log10(omega_din));
        Pmag=bode(Plant,w);
        for i=1:size(Pmag,3), Pmag_(i)=Pmag(1,1,i); end
        plot(w,20*log10(1/gamma_din)*ones(1,length(Pmag_))+20*log10(Pmag_),'g')

        %--- noise specification ---
        omega_n = 10^1;  % attenuate noise above this frequency
        gamma_n = .1;   % attenuate noise by this amount
        w = logspace(log10(omega_n),2+log10(omega_n));
        plot(w,20*log10(gamma_n)*ones(size(w)),'g')
        
%         %--- steady state tracking of step ---
%         gamma_0 = .01;
%         w = logspace(-5, 0);
%         plot(w,20*log10(1/gamma_0 -1)*ones(size(w)),'g')
        
        %--- steady state tracking of ramp ---
        gamma_1 = .03;
        w = logspace(-4, 0);
        plot(w,20*log10(1/gamma_1)-20*log10(w),'g')

%         %--- steady state tracking of parabola ---
%         gamma_2 = .01;
%         w = logspace(-5, 0);
%         plot(w,20*log10(1/gamma_2)-40*log10(w),'g')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% proportional control: change cross over frequency
     kp = 3;
     C = C*kp;
     %figure(1), margin(Plant*C), figure(2), bodemag(Plant*C) % update plot
     
% phase lag: add gain at low frequency (tracking, dist rejection)
     % low frequency gain = K*z/p
     % high frequency gain = K
     z = .01; % frequency at which lag ends
     M = 30;  % separation between pole and zero
     Lag = tf([1,z],[1,z/M]);
     C = C*Lag;
     %figure(1), margin(Plant*C), figure(2), bodemag(Plant*C) % update plot

% integral control: increase steady state tracking and dist rejection
     k_I = .4; % frequency at which integral action ends
     Integrator = tf([1,k_I],[1,0]);
     C = C*Integrator;
     %figure(1), margin(Plant*C), figure(2), bodemag(Plant*C) % update plot

% % phase lead: increase PM (stability)
%     wmax = 30; % location of maximum frequency bump
%     M    = 8; % separation between zero and pole
%     Lead =tf((1+sqrt(M))/(1+1/sqrt(M)))*[1,wmax/sqrt(M)],[1,wmax*sqrt(M)]);
%     C = C*Lead;
%     %figure(1), margin(Plant*C), figure(2), bodemag(Plant*C) % update plot

% low pass filter: decrease gain at high frequency (noise)
     p = 3;
     LPF = tf(p,[1,p]);
     C = C*LPF;
     %figure(1), margin(Plant*C), figure(2), bodemag(Plant*C) % update plot

% update plot
figure(1), margin(Plant*C), 
figure(2), bodemag(Plant*C)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
  F = tf(1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % notch filter: reduce closed loop peaking at cross over
%     ws = 1;  % frequency to start the notch
%     M  = 5;  % width of the notch
%     NOTCH = tf([1,2*sqrt(M)*ws,M*ws^2],[1,(M+1)*ws,M*ws^2]);
%     F = F*NOTCH;
 

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
        title('Closed Loop Bode Plot'), grid on
    subplot(3,1,2), 
        step(CLOSED_R_to_Y), hold on
        step(CLOSED_R_to_Y*F)
        title('Closed loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U), hold on
        step(CLOSED_R_to_U*F)
        title('Control effort for step response'), grid on
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[num,den] = tfdata(C,'v');
[P.A_C,P.B_C,P.C_C,P.D_C]=tf2ss(num,den);

[num,den] = tfdata(F,'v');
[P.A_F, P.B_F, P.C_F, P.D_F] = tf2ss(num,den);

% sample rate for controller
P.Ts = 0.01;

