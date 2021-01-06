function [sys,x0,str,ts,simStateCompliance]...
                = arm_dynamics(t,x,u,flag,P)
switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]...
                =mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u, P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error(...
        'Simulink:blocks:unhandledFlag',...
        num2str(flag));

end

% end sfuntmpl

%
%===============================================
% mdlInitializeSizes
%===============================================
function [sys,x0,str,ts,simStateCompliance]...
    =mdlInitializeSizes(P)

sizes = simsizes;

sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0; % system parameters
sizes.NumOutputs     = 3;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [P.theta0; P.thetadot0];


%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

simStateCompliance = 'UnknownSimState';
% end mdlInitializeSizes

%
%===============================================
% mdlDerivatives
% Return the derivatives for the continuous 
% states.
%===============================================
%
function sys=mdlDerivatives(t,x,u,P)
  theta    = x(1);
  thetadot = x(2);
  tau      = u(1);
  
  % system parameters randomly generated to make the 
  % system uncertain
  persistent m
  persistent ell
  persistent b
  persistent g
  if t==0
    alpha = 0.2;  % uncertainty parameter
    m = P.m * (1+2*alpha*rand-alpha);  % kg
    ell = P.ell * (1+2*alpha*rand-alpha); % m
    b = P.b * (1+2*alpha*rand-alpha); % N m 
    g = P.g; % m/s^2
  end
    
  thetaddot = (3/m/ell^2)*(tau...
      - b*thetadot - m*g*ell/2*cos(theta)); 

sys = [thetadot; thetaddot];

% end mdlDerivatives

%
%===============================================
% mdlUpdate
%===============================================
%
function sys=mdlUpdate(t,x,u)
sys = [];

% end mdlUpdate

%
%===============================================
% mdlOutputs
% Return the block outputs.
%===============================================
%
function sys=mdlOutputs(t,x,u)
    theta = x(1);

    sys = [theta; x];

% end mdlOutputs

%
%===============================================
% mdlGetTimeOfNextVarHit
%===============================================
function sys=mdlGetTimeOfNextVarHit(t,x,u)

% Example, set the next hit to be one second later.
sampleTime = 1; 
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%===============================================
% mdlTerminate
% Perform any end of simulation tasks.
%===============================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
