function tau=satellite_ctrl(in,P)
    phi_d   = in(1);
    theta   = in(2);
    phi     = in(3);
    t       = in(4);
    
    % digital differentiator for phidot and thetadot
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
    phidot = (2*P.sigma-P.Ts)...
        /(2*P.sigma+P.Ts)*phidot...
        + 2/(2*P.sigma+P.Ts)*(phi-phi_d1);
    thetadot = (2*P.sigma-P.Ts)...
        /(2*P.sigma+P.Ts)*thetadot...
        + 2/(2*P.sigma+P.Ts)*(theta-theta_d1);
    phi_d1 = phi;
    theta_d1 = theta;

    % construct the state
    x = [theta; phi; thetadot; phidot];
    % compute the state feedback controller
    tau = sat( -P.K*x + P.kr*phi_d, P.taumax);
end

%--------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end