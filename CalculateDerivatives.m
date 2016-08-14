function xdot = CalculateDerivatives(x,omega,paramStruct)
%CALCULATEDERIVATIVES Function to calculate the state derivatives of a
%multirotor micro-UAV.
%   
%
%
%% Extract constants
g = paramStruct.g;
% Inertial properties
m = paramStruct.m;
I = paramStruct.I;           
% Aerodynamic surface properties
numSurfaces = paramStruct.numSurfaces;
surfacePosn = paramStruct.surfacePosn;
surfaceDCM = paramStruct.surfaceDCM;
% Rotor properties
numRotors = paramStruct.numRotors;
KT = paramStruct.KT;
KQ = paramStruct.KQ;
tauT = paramStruct.tauT;
tauQ = paramStruct.tauQ;
% Extract rotor position wrt cg in body axes.
rotorPosn = paramStruct.rotorPosn;
rotorDCM = paramStruct.rotorDCM;
%
%% Extract the state vector
% 
u = x(1,1); v = x(2,1); w = x(3,1); p = x(4,1); q = x(5,1); r = x(6,1); 
phi = x(7,1); theta = x(8,1); psi = x(9,1); 
T1 = x(13,1); T2 = x(14,1); T3 = x(15,1); T4 = x(16,1);
Q1 = x(17,1); Q2 = x(18,1); Q3 = x(19,1); Q4 = x(20,1);
% note the NED positions are not required by the derivatives
% calculations.
% 
%% Forces and Moments
%
% Get the aerodynamic forces and moments
[Xa,Ya,Za,La,Ma,Na] = AerodynamicFM(x,omega,w,surfacePosn,surfaceDCM,numSurfaces,...
                                    'MultiRotor','L1');
%
% Now the propulsive forces and moments
[Xp,Yp,Zp,Lp,Mp,Np] = PropulsiveFM(x,omega,w,rotorPosn,rotorDCM,numRotors,...
                                  'MultiRotor','L1');
%
%% Kinematics
% Direction cosines...
C_tht = [cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
C_psi = [cos(psi) sin(psi) 0;-sin(psi) cos(psi) 0; 0 0 1];
C_phi = [1 0 0;0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
% Giving the Euler matrix,...
C_E2B = C_phi*C_tht*C_psi;
nv = C_E2B'*[u;v;w];        % navigation vector (xed,yed,zed)
gv = C_E2B*[0;0;m*g];       % gravity vector in body axes
%% Equations of motion
%
% Body axes translational accelerations (X,Y,Z)
udot = v*r-q*w +(Xa + Xp + gv(1,1))/m;
vdot = p*w-u*r +(Ya + Yp + gv(2,1))/m;
wdot = q*u-v*p +(Za + Zp + gv(3,1))/m;
% Body axes rotational accelerations (general inertia tensor)
om = [p;q;r];
H = I*om;
extTorques = [La + Lp; Ma + Mp; Na + Np];
pd = I\(extTorques - cross(om,H));
pdot = pd(1,1);
qdot = pd(2,1);
rdot = pd(3,1);
% Euler angle rates. 
phidot = p + tan(theta)*(q*sin(phi) + r*cos(phi));
thetadot = q*cos(phi) - r*sin(phi);
psidot = sec(theta)*(q*sin(theta) + r*cos(theta));
% Attitude Quaternion (can be used if required)
% q0 = -0.5*(p*q1 + q*q2 + r*q3);
% q1 = 0.5*(p*q0 +r*q2 - q*q3);
% q2 = 0.5*(q*q0 -r*q1 +p*q3);
% q3 = 0.5*(r*q0 +q*q1 -p*q2);
% True multirotor position (NED coordinates)
xedot = nv(1,1);
yedot = nv(2,1);
zedot = nv(3,1);
% Now the thrust dynamics
T1dot = (KT * omega(1,1) - T1 )/tauT;
T2dot = (KT * omega(2,1) - T2 )/tauT;
T3dot = (KT * omega(3,1) - T3 )/tauT;
T4dot = (KT * omega(4,1) - T4 )/tauT;
% and rotor torque equations
Q1dot = (KQ * omega(1,1) - Q1 )/tauQ;
Q2dot = (KQ * omega(2,1) - Q2 )/tauQ;
Q3dot = (KQ * omega(3,1) - Q3 )/tauQ;
Q4dot = (KQ * omega(4,1) - Q4 )/tauQ;
% Now pack all the derivative terms back into the return array
xdot = [udot vdot wdot pdot qdot rdot...
        phidot thetadot psidot xedot yedot zedot...
        T1dot T2dot T3dot T4dot Q1dot Q2dot Q3dot Q4dot]';
end

