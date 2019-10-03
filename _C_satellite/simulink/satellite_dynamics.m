function [sys,x0,str,ts,simStateCompliance]...
        = satellite_dynamics(t,x,u,flag,P)
switch flag
  % Initialization %
  case 0
    [sys,x0,str,ts,simStateCompliance]...
        =mdlInitializeSizes(P);
  % Derivatives %
  case 1
    sys=mdlDerivatives(t,x,u);
  % Update %
  case 2
    sys=mdlUpdate(t,x,u);
  % Outputs %
  case 3
    sys=mdlOutputs(t,x,u);
  % GetTimeOfNextVarHit %
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  % Terminate %
  case 9
    sys=mdlTerminate(t,x,u);
  % Unexpected flags %
  otherwise
    DAStudio.error(...
        'Simulink:blocks:unhandledFlag',...
        num2str(flag));

end

% end sfuntmpl

%
%================================================
% mdlInitializeSizes
%================================================
function [sys,x0,str,ts,simStateCompliance]...
    =mdlInitializeSizes(P)

sizes = simsizes;

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);

% initialize the initial conditions
x0  = [P.theta0; P.phi0; P.thetadot0; P.phidot0];

% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%================================================
% mdlDerivatives
% Return the derivatives for the continuous 
% states.
%================================================
function sys=mdlDerivatives(t,x,u)
  theta    = x(1);
  phi      = x(2);
  thetadot = x(3);
  phidot   = x(4);
  tau      = u(1);
  
  % system parameters randomly generated to 
  % make the system uncertain
  persistent Js
  persistent Jp
  persistent k
  persistent b
  if t==0
    alpha = 0.2;  % uncertainty parameter
    Js = 5; % kg m^2
    Jp = 1;  % kg m^2
    k = 0.15; % N m 
    b = 0.05; % N m s
  end
  
  M = [...
      Js, 0; 0, Jp;...
      ];
  c = [...
      tau - b*(thetadot-phidot)-k*(theta-phi);...
      -b*(phidot-thetadot)-k*(phi-theta);...
      ];

  tmp = inv(M)*c;
  thetaddot = tmp(1);
  phiddot   = tmp(2); 

sys = [thetadot; phidot; thetaddot; phiddot];

% end mdlDerivatives

%
%================================================
% mdlUpdate
%================================================
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%================================================
% mdlOutputs
% Return the block outputs.
%================================================
function sys=mdlOutputs(t,x,u)
  theta    = x(1);
  phi      = x(2);

sys = [theta; phi; x];

% end mdlOutputs

%
%================================================
% mdlGetTimeOfNextVarHit
%================================================
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%================================================
% mdlTerminate
% Perform any end of simulation tasks.
%================================================
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
