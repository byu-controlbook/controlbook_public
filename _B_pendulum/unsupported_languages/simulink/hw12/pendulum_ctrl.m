function F=pendulum_ctrl(in,P)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
    % digital differentiator - zdot, thetadot
    persistent zdot
    persistent z_d1
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at t=0
    if t<P.Ts,
        zdot        = 0;
        z_d1        = 0;
        thetadot    = 0;
        theta_d1    = 0;
    end
    zdot = (2*P.sigma-P.Ts)/...
        (2*P.sigma+P.Ts)*zdot...
        + 2/(2*P.sigma+P.Ts)*(z-z_d1);
    thetadot = (2*P.sigma-P.Ts)/...
        (2*P.sigma+P.Ts)*thetadot...
        + 2/(2*P.sigma+P.Ts)*(theta-theta_d1);
    z_d1 = z;
    theta_d1 = theta;
    
    % integrator
    error = z_r - z;
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

    % construct the state
    x = [z; theta; zdot; thetadot];
    % compute the state feedback controller
    F_unsat = -P.K*x - P.ki*integrator;
    F = sat( F_unsat, P.F_max);
    
    % integrator anti-windup
    if P.ki~=0,
       integrator = integrator...
           + P.Ts/P.ki*(F-F_unsat);
    end

end

%-----------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end