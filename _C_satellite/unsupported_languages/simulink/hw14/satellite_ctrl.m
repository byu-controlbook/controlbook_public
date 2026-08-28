function out=satellite_ctrl(in,P)
    phi_r     = in(1);
    y         = [in(2); in(3)];
    t         = in(4);
    
    persistent tau  % delayed input 
    if t<P.Ts
        tau    = 0;
    end
    [x_hat, d_hat] = update_observer(t, y, tau, P);
    phi_hat = x_hat(2);
    
    % integrator
    error = phi_r - phi_hat;
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
    tau_unsat = -P.K*x_hat...
        - P.ki*integrator - d_hat;
    tau = sat( tau_unsat, P.taumax);
    
        % integrator anti-windup
    if P.ki~=0
       integrator = integrator...
           + P.Ts/P.ki*(tau-tau_unsat);
    end
    
    out = [tau; x_hat];

end
%----------------------------------------
% observer
function [xhat, dhat] = update_observer(t, y, u, P)
    persistent obsv_state  
    if t<P.Ts
        obsv_state = [0;0;0;0;0];
    end
    % update observer using RK4 integration
    F1 = observer_f(obsv_state, y, u, P);
    F2 = observer_f(obsv_state + P.Ts/2*F1, y, u, P);
    F3 = observer_f(obsv_state + P.Ts/2*F2, y, u, P);
    F4 = observer_f(obsv_state + P.Ts*F3, y, u, P);
    obsv_state = obsv_state...
         + P.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
    xhat = obsv_state(1:4);
    dhat = obsv_state(5);
end
%----------------------------------------
function x_hat_dot = observer_f(obsv_state, y, u, P)
	x_hat_dot = P.A2 * obsv_state...
                + P.B1 * u...
                + P.L2 * (y - P.C2 * obsv_state);
end
%----------------------------------------
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end