% vel1 = GE420_serialread('arrVel1',1000,'COM5');
% vel2 = GE420_serialread('arrVel2',1000,'COM5');
% vel = [vel1; vel2];
% vel = GE420_serialread('arrVel1',400,'COM5');

%load('fivteenms.mat');
%load('fivems.mat');
load('onems.mat');
plot(vel);
velmax = max(vel);
disp('0.63*max is ');
disp(0.63*velmax);
K = velmax/5;

B = 5*ones(1999,1);
A = [vel(1:(end-1)) B];
b = vel(2:end);
C = A\b;
disp('c1');
disp(vpa(C(1),7));
disp('c2');
disp(vpa(C(2),7));

taum = vpa(-0.001/log(C(1)))
K_calc = vpa(C(2)/ (1 - C(1)))

tf1 = tf(C(2), [1 -1*C(1)], 0.005);


