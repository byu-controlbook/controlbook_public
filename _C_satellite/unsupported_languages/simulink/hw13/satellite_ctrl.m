function out=satellite_ctrl(in, P)
    phi_r     = in(1);
    y         = [in(2); in(3)];
    t         = in(4);
    
    persistent tau
    if t<P.Ts
        tau    = 0;
    end
    xhat = update_observer(t, y, tau, P);
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

function x_hat = update_observer(t, y, u, P)
    persistent xhat % estimated state
    if t<P.Ts
        xhat = [0;0;0;0];
    end
    % update observer using RK4 integration
    F1 = observer_f(xhat, y, u, P);
    F2 = observer_f(xhat + P.Ts/2*F1, y, u, P);
    F3 = observer_f(xhat + P.Ts/2*F2, y, u, P);
    F4 = observer_f(xhat + P.Ts*F3, y, u, P);
    xhat = xhat + P.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
    x_hat = xhat;
end

function x_hat_dot = observer_f(x_hat, y, u, P)
    x_hat_dot = P.A * x_hat...
                + P.B * u...
                + P.L * (y - P.C * x_hat);
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end