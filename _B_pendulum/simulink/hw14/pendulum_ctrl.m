function out=pendulum_ctrl(in,P)
    z_r     = in(1);
    z_m     = in(2);
    theta_m = in(3);
    t       = in(4);
    
    % implement observer
    persistent xhat       % estimated state 
    persistent dhat       % estimate disturbance
    persistent F          % delayed input 
    if t<P.Ts,
        xhat = [0;0;0;0];
        dhat = 0;
        F    = 0;
    end
    N = 10;
    for i=1:N,
        xhat = xhat + ...
            P.Ts/N*(P.A*xhat+P.B*(F+dhat)...
               +P.L*([z_m;theta_m]-P.C*xhat));
        dhat = dhat...
            + P.Ts/N*P.Ld*([z_m;theta_m]-P.C*xhat);
    end
    zhat = xhat(1);
    

    % integrator
    error = z_r - zhat;
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

    % compute the state feedback controller
    F_unsat = -P.K*xhat...
        - P.ki*integrator - dhat;
    F = sat( F_unsat, P.F_max);
    
    % integrator anti-windup
    if P.ki~=0,
       integrator = integrator...
           + P.Ts/P.ki*(F-F_unsat);
    end
    
    out = [F; xhat];

end

%-----------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end