function tau=arm_ctrl(in,P)
    theta_r = in(1);
    theta   = in(2);
    t       = in(3);
    
    % digital differentiator - thetadot
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at t=0
    if t<P.Ts
        thetadot    = 0;
        theta_d1    = 0;
    end
    thetadot = P.beta*thetadot...
        + (1-P.beta)*...
          ((theta-theta_d1)/P.Ts);
    theta_d1 = theta;
    
    % implement integrator
    error = theta_r - theta;
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

    
    % construct the state
    theta_e = 0;
    x = [theta-theta_e; thetadot];
    % compute equilibrium torque tau_e
    tau_e = P.m*P.g*(P.ell/2)*cos(theta);
    % compute total torque
    tau_unsat = tau_e - P.K*x...
        - P.ki*integrator;
    tau = sat( tau_unsat, P.tau_max);
    
    % integrator anti-windup
    if P.ki~=0
       integrator = integrator...
           + P.Ts/P.ki*(tau-tau_unsat);
    end
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end