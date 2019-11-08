function out=satellite_ctrl(in, P)
    phi_r     = in(1);
    y         = [in(2); in(3)];
    t         = in(4);
    
    % implement observer
    persistent xhat % estimated state
    persistent tau
    if t<P.Ts
        xhat = [0;0;0;0];
        tau    = 0;
    end
    N = 10;
    for i=1:N
        xhat = xhat + ...
            P.Ts/N*(P.A*xhat+P.B*tau+P.L*(y - P.C*xhat));
    end
    phihat = xhat(2);
    
    % integrator
    error = phi_r - phihat;
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
    tau_unsat = -P.K*xhat - P.ki*integrator;
    tau = sat( tau_unsat, P.taumax);
    
    % integrator anti-windup
    if P.ki~=0
       integrator = integrator...
           + P.Ts/P.ki*(tau-tau_unsat);
    end
    out = [tau; xhat];
end

%-----------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end