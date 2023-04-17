function P = loopshape_example

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
    % add_spec_tracking(0.001, 0.001);
    % add_spec_tracking_step(0.01);
    add_spec_tracking_ramp(0.03);
    % add_spec_tracking_parabola(0.010
    add_spec_disturbance(0.1, 0.1, Plant);
    add_spec_noise(0.1, 10);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control Design
    C = tf(1, 1);
    C = add_control_integral(C, 0.4);
    C = add_control_proportional(C, 3);
    C = add_control_lag(C, 0.01, 30);
    C = add_control_lpf(C, 3);
    % C = add_control_lead(C, 30, M);

    % update plot
    figure(1), margin(Plant * C), 
    figure(2), bodemag(Plant * C)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Prefilter Design
    F = tf(1, 1);
    % F = add_control_notch(F, 1, 5);
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create plots for analysis
    
    % Open-loop tranfer function 
      OPEN = Plant*C;
    % closed loop transfer function from R to Y
      CLOSED_R_to_Y = (Plant*C/(1+Plant*C));
    % closed loop transfer function from R to U
      CLOSED_R_to_U = (C/(1+C*Plant));

    figure(3), clf
        subplot(3,1,1), 
            bodemag(CLOSED_R_to_Y), hold on
            bodemag(CLOSED_R_to_Y * F)
            title('Closed Loop Bode Plot'), grid on
        subplot(3,1,2), 
            step(CLOSED_R_to_Y), hold on
            step(CLOSED_R_to_Y * F)
            title('Closed loop step response'), grid on
        subplot(3,1,3), 
            step(CLOSED_R_to_U), hold on
            step(CLOSED_R_to_U * F)
            title('Control effort for step response'), grid on

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Convert controller to state space equations for implementation
    [P.num_C, P.den_C] = tfdata(C,'v');
    [P.num_F, P.den_F] = tfdata(F,'v');
    % sample rate for controller
    P.Ts = 0.01;
end

%--- general tracking specification ---
% track references above omega_r by gamma_r
function add_spec_tracking(gamma_r, omega_r)
	w = logspace(log10(omega_r)-2,log10(omega_r));
	plot(w,20*log10(1/gamma_r)*ones(size(w)),'g')
end

%--- steady state tracking of step ---
% track step to within gamma_r
function add_spec_tracking_step(gamma_r)
    w = logspace(-5, 0);
    plot(w, 20*log10(1/gamma_r -1)*ones(size(w)),'g')
end

%--- steady state tracking of ramp ---
% track ramp to within gamma_r
function add_spec_tracking_ramp(gamma_r)
    w = logspace(-4, 0);
    plot(w,20*log10(1/gamma_r)-20*log10(w),'g')
end

%--- steady state tracking of parabola ---
% track ramp to within gamma_r
function add_spec_tracking_parabola(gamma_r)
    w = logspace(-5, 0);
    plot(w,20*log10(1/gamma_r)-40*log10(w),'g')
end

%--- input disturbance specification ---
% reject distubance below omega_d by gamma_d
function add_spec_disturbance(gamma_d, omega_d, Plant)
    w = logspace(log10(omega_d)-2, log10(omega_d));
    Pmag=bode(Plant,w);
    for i=1:size(Pmag,3), Pmag_(i)=Pmag(1,1,i); end
    plot(w,20*log10(1/gamma_d)*ones(1,length(Pmag_))+20*log10(Pmag_),'g')
end

%--- noise specification ---
% attenuate noise about omega_n by gamma_n
function add_spec_noise(gamma_n, omega_n)
	w = logspace(log10(omega_n),2+log10(omega_n));
	plot(w,20*log10(gamma_n)*ones(size(w)),'g')
end

% proportional control: change cross over frequency
% proportional gain kp
function Cnew = add_control_proportional(C, kp)
	Cnew = C*kp;
end

% integral control: increase steady state tracking and dist rejection
% ki: frequency at which integral action ends
function Cnew = add_control_integral(C, ki) 
	Integrator = tf([1, ki], [1, 0]);
	Cnew = C*Integrator;
end

% phase lag: add gain at low frequency (tracking, dist rejection)
% z: frequency at which lag ends
% M: separation between pole and zero
function Cnew = add_control_lag(C, z, M)
    Lag = tf([1, z], [1, z/M]);
	Cnew = C * Lag;
end

% low pass filter: decrease gain at high frequency (noise)
% p:  low pass filter frequency
function Cnew = add_control_lpf(C, p)
    LPF = tf(p,[1, p]);
    Cnew = C * LPF;
end

% phase lead: increase PM (stability)
% wmax: location of maximum frequency bump
% M: separation between zero and pole
function Cnew = add_control_lead(C, omega_L, M)
    gain = (1+sqrt(M))/(1+1/sqrt(M));
    Lead =tf(gain * [1, omega_L/sqrt(M)], [1, omega_L*sqrt(M)]);
	Cnew = C * Lead;
end

% notch filter: reduce closed loop peaking at cross over
% ws: frequency to start the notch
% M: width of the notch
function Cnew = add_control_notch(C, ws, M)
    Notch = tf([1,2*sqrt(M)*ws,M*ws^2],[1,(M+1)*ws,M*ws^2]);
	Cnew = Notch * C;
end
