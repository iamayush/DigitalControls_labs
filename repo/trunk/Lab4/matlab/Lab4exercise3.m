%% SERIAL READ
% initially 7 amp
time = GE420_serialread('arrTime',1000,'COM5');
vel = GE420_serialread('arrVel',1000,'COM5');
u = GE420_serialread('arrU',1000,'COM5');

%% Plotting
figure;
plot(time,u);
figure;
plot(time, vel);

%% Change AMP to 5
GE420_serialwrite('Amp',5,'COM5');
GE420_serialwrite('matlabLock',0,'COM5');

pause(100); 

time = GE420_serialread('arrTime',1000,'COM5');
vel = GE420_serialread('arrVel',1000,'COM5');
u = GE420_serialread('arrU',1000,'COM5');

%% Plotting
figure;
plot(time,u,'r');
figure;
plot(time, vel,'r');
