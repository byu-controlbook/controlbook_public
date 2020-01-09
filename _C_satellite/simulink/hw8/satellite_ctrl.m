function tau=satellite_ctrl(in,P)
    phi_r    = in(1);
    phi      = in(2);
    theta    = in(3);
    phidot    = in(4);
    thetadot = in(5);
    t        = in(6);
    
    % outer loop controller
    theta_r = P.kp_phi * (phi_r - phi) - P.kd_phi * phidot;
    % inner loop controller
    tau = P.kp_th * (theta_r - theta) - P.kd_th * thetadot;
    tau = saturate(tau, P.taumax);
end
%---------------------------------------------------------
% saturation function
function out = saturate(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end