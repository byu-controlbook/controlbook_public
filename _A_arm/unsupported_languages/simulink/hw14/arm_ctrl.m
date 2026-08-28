function out=arm_ctrl(in,P)
    theta_r = in(1);
    theta_m   = in(2);
    t       = in(3);
    
    % implement observer
    persistent xhat       % estimated state
    persistent dhat       % estimate disturbance
    persistent tau        % delayed input
    if t<P.Ts
        xhat  = [0; 0];
        dhat  = 0;
        tau   = 0;
    end
    % compute equilibrium torque tau_e
    theta_e = 0;
    theta = xhat(1);
    tau_e = P.m*P.g*(P.ell/2)*cos(theta);
    x_e = [theta_e; 0];
    N = 10;
    for i=1:N
        xhat = xhat + ...
            P.Ts/N*(P.A*(xhat-x_e)...
                +P.B*(tau-tau_e+dhat)...
                +P.L*(theta_m-P.C*xhat));
        dhat = dhat...
            + P.Ts/N*P.Ld*(theta_m-P.C*xhat);
    end

    % add integrator
    error = theta_r - theta;
    persistent integrator
    persistent error_d1
    % reset persistent variables at t=0
    if t<P.Ts==1
        integrator  = 0;
        error_d1    = 0;
    end
    integrator = integrator...
        + (P.Ts/2)*(error+error_d1);
    error_d1 = error;

    % compute the state feedback controller
    tau_unsat = tau_e-P.K*(xhat-x_e)...
        -P.ki*integrator-dhat;
    tau = sat( tau_unsat, P.tau_max);
    
    % integrator anti-windup
    if P.ki~=0
       integrator = integrator...
           + P.Ts/P.ki*(tau-tau_unsat);
    end

    out = [tau; xhat];
    
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end