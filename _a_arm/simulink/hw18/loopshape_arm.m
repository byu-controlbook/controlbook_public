
% start with pid control designed in problem 10
 figure(2), clf
    bodemag(Plant*C_pid,logspace(-3,5))
    hold on
    grid on
    
% add constraints
        
% increase tracking by factor of 10 below 
%   omega_r=0.007
  % reject input dist. below this frequency
  omega_r = 0.07;  
  % amount of input disturbance in output
  gamma_r = 0.1;    
  w = logspace(log10(omega_r)-2,log10(omega_r));
  Pmag=bode(Plant*C_pid,w);
  for i=1:size(Pmag,3), Pmag_(i)=Pmag(1,1,i); end
  plot(w,20*log10(1/gamma_r)...
           *ones(1,length(Pmag_))...
         +20*log10(Pmag_),'g')    

% attenuate noise by factor of 
%   gamma_n above omega_n
  % attenuate noise above this frequency
  omega_n = 1000;  
  % amount of noise attenuation in output
  gamma_n = 0.1;    
  w = logspace(log10(omega_n),log10(omega_n)+2);
  Pmag=bode(Plant*C_pid,w);
  for i=1:size(Pmag,3), Pmag_(i)=Pmag(1,1,i); end
  plot(w,20*log10(gamma_n)...
           *ones(1,length(Pmag_))...
         +20*log10(Pmag_),'g')
  figure(2), margin(Plant*C_pid)
  %pause % hw_arm_compensator_design_1
              
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Control Design
     C = C_pid;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% phase lag (|p|<|z|): add gain at low frequency 
%                      (tracking, dist rejection)
     % low frequency gain = K*z/p
     % high frequency gain = K
     z = .7;
     p = z/10;
     Lag = tf([1,z],[1,p]);
     C = C*Lag;
     figure(2), margin(Plant*C)
     %pause % hw_arm_compensator_design_2
     
% phase lead (|p|>|z|): increase PM (stability)
     % low frequency gain = K*z/p
     % high frequency gain = K
     % location of maximum frequency bump
     wmax = 30; 
     M   = 10; % separation between zero and pole
     Lead =tf(M*[1,wmax/sqrt(M)],[1,wmax*sqrt(M)]);
     C = C*Lead;
     figure(2), margin(Plant*C)
     %pause % hw_arm_compensator_design_3
     
% low pass filter: 
%    decrease gain at high frequency (noise)
     p = 50;
     LPF = tf(p,[1,p]);
     C = C*LPF;
     figure(2), margin(Plant*C)
     %pause % hw_arm_compensator_design_4

% low pass filter: 
%    decrease gain at high frequency (noise)
     p = 150;
     LPF = tf(p,[1,p]);
     C = C*LPF;
     figure(2), margin(Plant*C)
     %pause % hw_arm_compensator_design_5

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% add a prefilter to eliminate the overshoot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = 1;
% low pass filter
     p = 3;
     LPF = tf(p,[1,p]);
     F = F*LPF;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open-loop transfer function 
OPEN = Plant*C;
% closed loop transfer function from R to Y
CLOSED_R_to_Y = (Plant*C/(1+Plant*C));
% closed loop transfer function from R to U
CLOSED_R_to_U = (C/(1+C*Plant));

figure(3), clf
    subplot(1,3,1), 
        bodemag(CLOSED_R_to_Y), hold on
        bodemag(CLOSED_R_to_Y*F)
        title('Closed Loop Bode Plot'), grid on
    subplot(1,3,2), 
        step(CLOSED_R_to_Y), hold on
        step(CLOSED_R_to_Y*F)
        title('Closed loop step response'), 
        grid on
    subplot(1,3,3), 
        step(CLOSED_R_to_U), hold on
        step(CLOSED_R_to_U*F)
        title('Control effort for step response'), 
        grid on
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[num,den] = tfdata(C,'v');
[P.A_C, P.B_C, P.C_C, P.D_C]=tf2ss(num,den);

[num,den] = tfdata(F,'v');
[P.A_F, P.B_F, P.C_F, P.D_F] = tf2ss(num,den);

