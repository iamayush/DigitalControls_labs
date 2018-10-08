% vel = GE420_serialread('arrVel1',400,'COM5');

load('fivems.mat');
load('tf1.mat');

plot(vel);
velmax = max(vel);
disp('0.63*max is ');
disp(0.63*velmax);
K = velmax/5;

B = 5*ones(397,1);
A = [-1*vel(3:(end-1)) -1*vel(2:(end-2)) -1*vel(1:(end-3)) B B B];
b = vel(4:end);
C = A\b;
disp('c1');
disp(vpa(C(1),7));
disp('c2');
disp(vpa(C(2),7));
disp('c3');
disp(vpa(C(3),7));
disp('c4');
disp(vpa(C(4),7));
disp('c5');
disp(vpa(C(5),7));
disp('c6');
disp(vpa(C(6),7));

mytf = tf([C(6) C(5) C(4)],[1 C(1) C(2) C(3)],0.005);
figure
step(tf1,mytf);
legend('tf1','mytf');
figure
bode(tf1,mytf);
legend('tf1','mytf');

taum = vpa(-0.001/log(C(1)))
K_calc = vpa(C(2)/ (1 - C(1)))



