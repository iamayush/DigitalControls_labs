%% Continuous Time Plant
K=0.325; %Gain
w=2*pi*10; %frequency (wn in rad/s)
ze=0.001; %zeta - damping ratio

planttf=tf([K*w^2],[1 2*ze*w w^2]); %continuous plant

%% Discrete Time Plant 
plantdtf=c2d(planttf,0.001,'zoh'); %discrete plant using ZOH

%% Controller Design
pidtf=tf([1 0],[1 -1],0.001); %PI discrete tf 
Dtf = tf([500 0],[1 500]);  %D continuous tf
Ddtf = c2d(Dtf,0.001,'tustin') %D Discrete tf
