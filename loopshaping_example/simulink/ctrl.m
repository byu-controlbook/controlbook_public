function u=ctrl(in,P)
    y_d    = in(1);
    y      = in(2);
    t      = in(3);
    
    % define and initialize persistent variables
    persistent z  % state of the controller
    if t==0,
        z = zeros(size(P.A_C,1),1);
    end
    
    % error signal
    error = y_d - y;
    
    % solve differential equation defining controller
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        z = z + P.Ts/N*( P.A_C*z + P.B_C*error );
    end
    % output equation for the controller
    u = P.C_C*z + P.D_C*error;
    
end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end