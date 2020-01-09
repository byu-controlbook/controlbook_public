function tau=satellite_ctrl(in,P)
    phi_r     = in(1);
    theta_m   = in(2);
    phi_m     = in(3);
    t         = in(4);

    % initialize differentiators for phi and theta
    persistent phidot
    persistent phi_d1
    persistent thetadot
    persistent theta_d1
    if t<P.Ts
        phidot = 0;
        phi_d1 = 0;
        thetadot = 0;
        theta_d1 = 0;
    end
 
    % initialize controller state z
    persistent xout_C
    persistent xout_F
    persistent xin_C
    if t<P.Ts
          xout_C = zeros(size(P.Aout_C,1),1);
          xout_F = zeros(size(P.Aout_F,1),1); 
          xin_C  = zeros(size(P.Ain_C,1),1);
    end
   % update derivative of phi and theta
    phidot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*phidot...
             + 2/(2*P.sigma+P.Ts)*(phi_m-phi_d1);
    thetadot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*thetadot...
              + 2/(2*P.sigma+P.Ts)*(theta_m-theta_d1);
    phi_d1 = phi_m;
    theta_d1 = theta_m;
  
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        xout_F = xout_F + P.Ts/N*( P.Aout_F*xout_F + P.Bout_F*phi_r );
        % output equation for the prefilter
        phi_r_filtered = P.Cout_F*xout_F + P.Dout_F*phi_r;
        % error signal for outer loop
        error_out = phi_r_filtered - phi_m;
        xout_C = xout_C + P.Ts/N*( P.Aout_C*xout_C + P.Bout_C*error_out );
        % output equation for the controller
        theta_r = -P.kd_phi*phidot + P.Cout_C*xout_C + P.Dout_C*error_out;

        % error signal for inner loop
        error_in = theta_r - theta_m;
        % state space equations for C
        xin_C = xin_C + P.Ts/N*( P.Ain_C*xin_C + P.Bin_C*error_in );
        % output equation for the controller
        tau = sat(  -P.kd_th*thetadot + P.Cin_C*xin_C + P.Din_C*error_in, P.taumax);
    end
  
end

%--------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end