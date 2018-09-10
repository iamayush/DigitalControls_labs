% This M-file gets you started in doing identification of the Inertia Wheel
% linkage.  This file calculates the parameters of the linkage by making
% some assumptions and then calculating the parameters.  Most of the
% parts of the Inertia Wheel I was able to weigh to get a mass value.
% both Center of Mass and Moments of Inertia are calculated by hand here in
% the script file.  These parameters will work to control the Inertia Wheel.

Imotorshaft = 9.8839e-7/4; %Kg-m^2   Guess.  a 1/4 of Reaction Wheel motor
mmotorshaft = 2*((((.003175)^2)/4)*pi*0.0635*7804.937); % Kg    guess - mass of motor's roter
% just a guess taking the mass of 2 motor shafts as a guess of the motor mass
                  % This is a guess and may be in error.  I am assuming that
                  % most of the mass of the motor is in the coil and since that
                  % is part of the torque generation unit the coil's mass is not
                  % included in the mass of link one.  So here I am considering
                  % the mass added to link one by the motor is just a steel shaft of
                  % length .0635m (2.5in) and diameter .003175m (0.125in).   Actual shaft diameter is 2.3 mm
%mhub1 = 0.0065;  % Kg  mass of link one's mounting hub
ml1p = 0.1157;  % Kg mass of link one's leg
mweight = 0.0529;
m_mot = 0.057-mmotorshaft;  % Kg mass of the Pittman motor and optical encoder minus the guess at the motor shaft's mass
Imot = (m_mot/8)*0.0254^2;  % Kg-m^2  guess   Moment of Inertia of the Pittman Motor and Encoder
mflywheel = 0.0578;  % Kg mass of flywheel with mounting hub

% I of link one's mounting hub about its center
%Ihub1 = (mhub1/8)*(0.013462^2+ 0.00635^2);

% I of link one's leg about its centroid
Il1p = (ml1p/12)*(0.04318^2 + 0.12192^2);   %1.7" x 4.8"  guess

% I of the flywheel
Iflywheel = (mflywheel/8)*(0.04445^2 + 0.005^2);

Iweight = (mweight/8)*(0.04445^2 + 0.005^2);

% Total mass of link one
ml1 = ml1p + m_mot + mweight;
% center of mass of link one
lc1 = (ml1p*0.08128 + m_mot*0.08128 + mweight*0.04318)/ml1;
% Total moment of Inertia of Link one about its center of mass
Il1 = Iweight + mweight*(lc1-0.04318)^2 + Il1p + ml1p*(lc1 - 0.08128)^2;
Il1 = Il1 + Imot + m_mot*(lc1-0.08128)^2;

% Total mass of link two
ml2 = mmotorshaft + mflywheel;
% Total moment of Inertia of Link Two about its center of mass
Il2 = Imotorshaft + Iflywheel;

% generate the five parameters
P(1) = ml1*lc1^2 + ml2*0.08128^2 + Il1;
P(2) = Il2;
P(3) = ml1*lc1 + ml2*0.08128;

% add the Torque Constant to the parameters
TorqueConst = 0.00206; %N-m/PWM command where pwm command is from -20 to 20 with 0 50% duty cycle
P = P/TorqueConst


