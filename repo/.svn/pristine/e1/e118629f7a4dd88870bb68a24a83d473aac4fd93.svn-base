function [A,B] = linRwhl(P,x1)
%
%  Format: [A,B] = linRwhl(P,x1)
%
%  This function finds the linear model for the Reaction wheel experiment about the operating
%  point {th,0,0}.  Where the states are x1,dx1/dt=x2 and dx3/dt=x4.
%  Where x1 is the encoder reading of the link angle and x3 is the
%  sum of the encoder reading of the link and the encoder reading of motor.
%  This function uses SI units so g is equal to 9.804 m/s^2.
%
% Inputs:
%    1. P:  vector containing Reaction Wheel parameters:
%                P(1) =  m1*lc1^2 + m2*l1^2 + I1
%                P(2) =  I2
%                P(3) =  m1*lc1 + m2*l1
%
%    2. x1 is the desired operating point of link one.  The two possible values are 0 or pi
%
% Outputs:
%    1. A and B the matrices in the linear equation for the Reaction Wheel
%          delta_xdot = A*delta_x + B*delta_tau;

allok = 1;
if nargin ~= 2
   allok = 0;
   disp('linRwhl needs 2 input arguments');
end
if nargout ~= 2
   allok = 0;
   disp('linRwhl needs 2 output variables');
end
if allok == 1
  [m,n] = size(P);
  if ((m ~= 3 | n ~= 1) & (n ~= 3 | m ~= 1))
     allok = 0;
     disp('The dimension of P is incorrect; P must have 3 elements');
  end
end
if allok == 1
  [m,n] = size(x1);
  if (m ~= 1 | n ~= 1)
     allok = 0;
     disp('The dimension of x1 is incorrect; x1 must be a scaler');
  end
end

if allok == 1
   g=9.804;
   df2dx1=(-g*P(3)*cos(x1))/P(1);
   df2dx3=0;
   df4dx1=0;
   df4dx3=0;
   df2du=-1/P(1);
   df4du=1/P(2);
   A(2,1) = df2dx1;
   A(2,3) = df2dx3;
   A(3,1) = df4dx1;
   A(3,3) = df4dx3;
   B(2,1) = df2du;
   B(3,1) = df4du;
   A(1,2) = 1;
else
   disp('Error in variables entered type "help linRwhl" for more info');
end
