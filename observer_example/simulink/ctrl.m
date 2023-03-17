function out=ctrl(in,P)
    r      = in(1);
    y_m    = in(2);
    t      = in(3);
    x      = in(4:5);
    
    % define and initialize persistent variables
    persistent integrator
    persistent error_d1
    persistent xhat        % estimated state (for observer)
    persistent dhat        % disturbance estimate
    persistent u           % delayed input (for observer)
    
    N = 10; % number of integration steps for each sample
    
    % compute state feedback control 
    switch P.control_mode,
        case 1, % no integrator, no disturbance estimate
            % initialize persistent variables
            if t==0,
                xhat  = zeros(2,1);
                u      = 0;
            end
            % solve observer differential equations
            for i=1:N,
                xhat = xhat + P.Ts/N*(P.A*xhat + P.B*u...
                                      +P.L*(y_m-P.C*xhat));
            end
            % feedback controller
            u = P.kr*r - P.K*xhat;
            
            
        case 2, % integrator, no disturbance estimate
            % initialize persistent variables
            if t==0,
                xhat  = zeros(2,1);
                u      = 0;
                integrator = 0;
                error_d1   = 0;
            end
            % solve observer differential equations
            for i=1:N,
                xhat = xhat + P.Ts/N*(P.A*xhat+P.B*u...
                                      +P.L*(y_m-P.C*xhat));
            end
            yhat = P.C*xhat;
            % implement integrator
            error = r - yhat;
            integrator = integrator + (P.Ts/2)*(error+error_d1);
            error_d1 = error;
            % feedback controller
            u = - P.K*xhat - P.ki*integrator;
            
        case 3, % no integrator, disturbance estimate
            % initialize persistent variables
            if t==0,
                xhat  = zeros(2,1);
                dhat   = 0;
                u      = 0;
            end
            % solve observer differential equations
            for i=1:N,
                xhat = xhat + P.Ts/N*(P.A*xhat + P.B*(u+dhat)...
                                      +P.L*(y_m-P.C*xhat));
                dhat  = dhat  + P.Ts/N*P.Ld*(y_m-P.C*xhat);
            end
            % feedback controller
            u = P.kr*r - P.K*xhat - dhat;
            
        case 4, % integrator and disturbance estimate
            % initialize persistent variables
            if t==0,
                xhat  = zeros(2,1);
                dhat   = 0;
                u      = 0;
                integrator = 0;
                error_d1   = 0;
            end
            % solve observer differential equations
            for i=1:N,
                xhat = xhat + P.Ts/N*(P.A*xhat + P.B*(u+dhat)...
                                      +P.L*(y_m-P.C*xhat));
                dhat  = dhat  + P.Ts/N*P.Ld*(y_m-P.C*xhat);
            end
            yhat = P.C*xhat;
            % implement integrator
            error = r-yhat;
            integrator = integrator + (P.Ts/2)*(error+error_d1);
            error_d1 = error;
            % feedback controller
            u = -P.K*xhat - P.ki*integrator - dhat;
     end
   
    out = [u; xhat];
end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end