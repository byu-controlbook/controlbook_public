function tau=arm_ctrl(in,P)
    theta_c = in(1);
    theta   = in(2);
    t       = in(3);
    
    % digital differentiator for thetadot
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of 
    % simulation
    if t<P.Ts
        thetadot    = 0;
        theta_d1    = 0;
    end
    thetadot = P.beta*thetadot ...
        + (1-P.beta)*((theta-theta_d1)/P.Ts);
    theta_d1 = theta;
    
    % construct the state
    x = [theta; thetadot];
    % compute equilibrium torque tau_e
    theta_e = 0.0;
    tau_e = P.m*P.g*(P.ell/2)*cos(theta_e);   
    % compute the state feedback controller
    tau_tilde = - P.K*x + P.kr*theta_c;
    % compute total torque
    tau = sat( tau_e + tau_tilde, P.tau_max);
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end