function tau=satellite_ctrl(in,P)
    phi_r   = in(1);
    theta   = in(2);
    phi     = in(3);
    t       = in(4);
    
    % use a digital differentiator to find phidot and thetadot
    persistent phidot
    persistent phi_d1
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        phidot        = 0;
        phi_d1        = 0;
        thetadot    = 0;
        theta_d1    = 0;
    end
    phidot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*phidot...
        + 2/(2*P.sigma+P.Ts)*(phi-phi_d1);
    thetadot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*thetadot...
        + 2/(2*P.sigma+P.Ts)*(theta-theta_d1);
    phi_d1 = phi;
    theta_d1 = theta;
    
    % integrator
    error = phi_r - phi;
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if t<P.Ts==1
        integrator  = 0;
        error_d1    = 0;
    end
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    error_d1 = error;    

    % construct the state
    x = [theta; phi; thetadot; phidot];
    % compute the state feedback controller
    tau_unsat = -P.K*x - P.ki*integrator;
    tau = sat( tau_unsat, P.taumax);
    
        % integrator anti-windup
    if P.ki~=0
       integrator = integrator + P.Ts/P.ki*(tau-tau_unsat);
    end

end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end