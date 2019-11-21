% satellite parameter file
addpath ./.. % adds the parent directory to the path
satelliteParam % general satellite parameters

% transfer functions
P_in = tf([1/P.Js],[1,P.b/P.Js,P.k/P.Js]);
P_out = tf([P.b/P.Jp,P.k/P.Jp],[1,P.b/P.Jp,P.k/P.Jp]);

% display bode plots of transfer functions
figure(2), clf, bode(P_in), grid on
figure(3), clf, bode(P_out), grid on




