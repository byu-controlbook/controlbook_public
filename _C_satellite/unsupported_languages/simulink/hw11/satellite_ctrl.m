function tau=satellite_ctrl(in,P)
    phi_d   = in(1);
    theta   = in(2);
    phi     = in(3);
    thetadot = in(4);
    phidot = in(5);
    t       = in(6);
    
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