function [sys,x0,str,ts,simStateCompliance]...
    = pendulum_dynamics(t,x,u,flag,P)
switch flag
  % Initialization
  case 0
    [sys,x0,str,ts,simStateCompliance]...
        =mdlInitializeSizes(P);
  % Derivatives
  case 1
    sys=mdlDerivatives(t,x,u,P);
  % Update 
  case 2
    sys=mdlUpdate(t,x,u);
  % Outputs 
  case 3
    sys=mdlOutputs(t,x,u);
  % GetTimeOfNextVarHit 
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

%
%==============================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and 
%  sample times for the S-function.
%==============================================
%
function [sys,x0,str,ts,simStateCompliance]...
    =mdlInitializeSizes(P)

sizes = simsizes;

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2+4;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);

% initialize the initial conditions%
x0  = [P.z0; P.theta0; P.zdot0; P.thetadot0];
% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

%
%==============================================
% mdlDerivatives
% Return the derivatives for the continuous 
% states.
%==============================================
%
function sys=mdlDerivatives(t,x,u,P)
  z        = x(1);
  theta    = x(2);
  zdot     = x(3);
  thetadot = x(4);
  F        = u(1);
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % The parameters for any physical system are 
  % never known exactly.  Feedback systems need to 
  % be designed to be robust to this uncertainty.  
  % In the simulation we model uncertainty by 
  % changing the physical parameters by a uniform 
  % random variable that represents alpha*100 % 
  % of the parameter, i.e., alpha = 0.2, means 
  % that the parameter may change by up to 20%.  
  % A different parameter value is chosen every 
  % time the simulation is run.
  persistent m1
  persistent m2
  persistent ell
  persistent b
  persistent g
  if t==0
    alpha = 0.0;  % uncertainty parameter
    m1 = P.m1 * (1+2*alpha*rand-alpha);  % kg
    m2 = P.m2 * (1+2*alpha*rand-alpha);     % kg
    ell = P.ell * (1+2*alpha*rand-alpha); % m
    b = P.b * (1+2*alpha*rand-alpha); % N m
    g = P.g; % the gravity vector is well known 
             % and so we don't change it.
  end

  % define system dynamics xdot=f(x,u)  
  M = [...
      m1+m2,              m1*ell/2*cos(theta);...
      m1*ell/2*cos(theta),  m1*ell^2/4;...
      ];
  c = [...
      m1*ell/2*thetadot^2*sin(theta)...
        + F - b*zdot;...
      m1*g*ell/2*sin(theta);...
      ];
  tmp = M\c;
  zddot     = tmp(1);
  thetaddot = tmp(2); 

sys = [zdot; thetadot; zddot; thetaddot];

% end mdlDerivatives

%
%==============================================
% mdlUpdate
% Handle discrete state updates, sample time 
% hits, and major time step requirements.
%==============================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%==============================================
% mdlOutputs
% Return the block outputs.
%==============================================
%
function sys=mdlOutputs(t,x,u)
    z        = x(1);
    theta    = x(2);
    % add Gaussian noise to the outputs
    z_m = z;% + 0.01*randn;
    theta_m = theta;% + 0.001*randn;
sys = [z_m; theta_m; x];

% end mdlOutputs

%
%==============================================
% mdlGetTimeOfNextVarHit
%==============================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%==============================================
% mdlTerminate
% Perform any end of simulation tasks.
%==============================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
