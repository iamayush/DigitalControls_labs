%% Continuous Systems
A = [0 1 0 0;
    0 0 -52.0624 0;
    0 0 0 1;
    0 0 76.1771 0];
B = [0;34.4136;0;-13.3529];

C = [1 0 0 0; 0 0 1 0];
D = [0;0];
ContSYS = ss(A,B,C,D);

%% Discrete System
DiscSYS = c2d(ContSYS,0.005,'zoh');

%% Placing poles
cp = [-30.1 -10 -7.2 -8.7279];
zp = exp(cp*0.005);
%K = place(DiscSYS.A,DiscSYS.B,zp);

%% Derivative approx
%ctf = tf([100 0],[1 100]);
%difftf = c2d(ctf,0.005,'tustin');

%% Observer 
ocp = 20*cp;
ozp = exp(ocp*0.005);
L = place(DiscSYS.A',DiscSYS.C',ozp)'
Kd = [-8.5445,-3.3264,-94.3109,-12.4099]

Ad = DiscSYS.A;
Bd = DiscSYS.B;
Cd = DiscSYS.C;
Dd = DiscSYS.D;

matrixtoCformat(Ad-L*Cd)
matrixtoCformat([Bd L])

