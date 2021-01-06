function out=arm_ctrl(in,P)
    theta_r = in(1);
    theta_m = in(2);
    t       = in(3);
    
    % implement observer
    persistent xhat   % estimated state
    persistent tau    % delayed input
    if t<P.Ts
        xhat  = [0; 0];
        tau   = 0;
    end
    N = 10;
    % compute equilibrium torque at thetahat
    tau_e = P.m*P.g*(P.ell/2)*cos(xhat(1)); 
    for i=1:N,
        xhat = xhat + ...
            P.Ts/N*(P.A*xhat+P.B*(tau-tau_e)...
                    +P.L*(theta_m-P.C*xhat));
    end
    thetahat = xhat(1);

    % implement integrator
    error = theta_r - thetahat;
    persistent integrator
    persistent error_d1
    % reset persistent variables at t=0
    if t<P.Ts==1,
        integrator  = 0;
        error_d1    = 0;
    end
    integrator = integrator...
        + (P.Ts/2)*(error+error_d1);
    error_d1 = error;

    % compute control torque
    tau_e = P.m*P.g*(P.ell/2)*cos(thetahat);     
    tau_unsat = tau_e - P.K*xhat...
        - P.ki*integrator;
    tau = sat( tau_unsat, P.tau_max);
    
    % integrator anti-windup
    if P.ki~=0,
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