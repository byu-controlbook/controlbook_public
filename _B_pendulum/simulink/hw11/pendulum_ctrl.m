function F=pendulum_ctrl(in,P)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
    % digital differentiator for 
    %  zdot and thetadot
    persistent zdot
    persistent z_d1
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of 
    %  simulation
    if t<P.Ts,
        zdot        = 0;
        z_d1        = 0;
        thetadot    = 0;
        theta_d1    = 0;
    end
    zdot = P.beta*zdot...
        + (1-P.beta)*(z-z_d1)/P.Ts;
    thetadot = P.beta*thetadot ...
        + (1-P.beta)*(theta-theta_d1)/P.Ts;
    z_d1 = z;
    theta_d1 = theta;

    % construct the state
    x = [z; theta; zdot; thetadot];
    % compute the state feedback controller
    F = sat( -P.K*x + P.kr*z_r, P.F_max);
end

%-------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end