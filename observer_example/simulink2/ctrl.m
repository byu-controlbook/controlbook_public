function out = ctrl(in, P)
    yd = in(1);
    y  = in(2);
    t  = in(3);

    persistent integrator
    persistent error_d1
    persistent xhat
    persistent dhat
    persistent u

    if t==0,
        integrator = 0;
        error_d1   = 0;
        xhat       = [0;0];
        dhat       = 0;
        u          = 0;
    end
    error = y-yd;
    integrator = integrator + P.Ts/2*(error+error_d1);
    error_d1 = error;
    
    
    N = 3;
    for i=1:N,
      xhat = xhat + P.Ts/N*(P.A*xhat + P.B*(u+dhat) + P.L*(y-P.C*xhat));
      dhat = dhat + P.Ts/N*P.Ld*(y-P.C*xhat);
    end

    u = -P.K*xhat + P.kr*yd - P.kI*integrator - dhat;
    
    out = [u; xhat];

end