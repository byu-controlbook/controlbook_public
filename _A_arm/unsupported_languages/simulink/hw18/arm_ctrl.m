function tau=arm_ctrl(in,P)
    theta_c = in(1);
    theta   = in(2);
    t       = in(3);
    
    % initialize controller state z
    persistent z_C
    persistent z_F
    if t<P.Ts,
          z_C = zeros(size(P.A_C,1),1);
          z_F = zeros(size(P.A_F,1),1);          
    end
    
    % prefilter the reference command
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        z_F = z_F + P.Ts/N*( P.A_F*z_F + P.B_F*theta_c );
    end
    % output equation for the prefilter
    theta_c_filtered = P.C_F*z_F + P.D_F*theta_c;
    
    % error signal
    error = theta_c_filtered - theta;

   
    
    % solve differential equation defining controller
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        z_C = z_C + P.Ts/N*( P.A_C*z_C + P.B_C*error );
    end
    % output equation for the controller
    tau_tilde = P.C_C*z_C + P.D_C*error;
    
    % compute equilibrium torque tau_e
    tau_e = P.m*P.g*(P.ell/2)*cos(theta);
    % compute total torque
    tau = tau_e + tau_tilde;    
end