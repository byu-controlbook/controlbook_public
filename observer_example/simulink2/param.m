clear all

P.A = [1 2; 3 4];
P.B = [5; 6];
P.C = [7, 8];

% sample rate
P.Ts = 0.01;

% transfer function
[num, den] = ss2tf(P.A,P.B,P.C,0);

zeta = 1.1;%0.707;
wn = 0.5;
des_poles = roots([1,2*zeta*wn, wn^2]);
int_pole = -.1;

A1 = [P.A, zeros(2,1); P.C, 0];
B1 = [P.B; 0];

K1 = place(A1,B1,[des_poles;int_pole]);
P.K = K1(1:2);
P.kI = K1(3);
P.kr = -1/(P.C*inv(P.A-P.B*P.K)*P.B);

zeta = 0.707;
wn = 5;
des_obs_poles = roots([1,2*zeta*wn, wn^2]);
dist_pole = -1;

A2 = [P.A, P.B; zeros(1,2), 0];
C2 = [P.C, 0];

L2 = place(A2',C2',[des_obs_poles; dist_pole])';
P.L = L2(1:2);
P.Ld = L2(3);
