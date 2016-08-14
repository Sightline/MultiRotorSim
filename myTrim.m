function [xtrim, utrim] = myTrim(x0, u0, airSpeed, climbAngle, sideSlip,...
                                 turnRate,paramStruct)
%MYTRIM Algorithm to return the trim state and control vectors for the
%aircraft for a specific flight condition.
%   Detailed explanation goes here
%
%
%
%   Author: Dr David Anderson
%   Date:   12/10/2015
%
%% Package data
% Package the function inputs into a data structure
trimData.Vf = airSpeed;
trimData.gamma = climbAngle;
trimData.beta = sideSlip;
trimData.turnRate = turnRate;
trimData.paramStruct = paramStruct;
%% Solve the trim optimisation
trimVariables = [1,1,1,1,0,0];
maxfun = 5000;
options = optimset('LargeScale','off',...
                   'Display','iter',...
                   'MaxIter',1000,...
                   'MaxFunEvals',maxfun,...
                   'TolFun',1e-9,...
                   'TolX',1e-9);
objFun = @(trimVariables)trimObjectiveFunction(trimVariables,trimData);
[xt,~,~,~] = fminunc(objFun,trimVariables,options);
%
%% Reconstruct the states & controls
% Now the control data
omega(1,1) = xt(1);
omega(2,1) = xt(2);
omega(3,1) = xt(3);
omega(4,1) = xt(4);
phi = xt(5);
tht = xt(6);
%% Constrain the states
% Here we need to use all the information contained within the trim state
% specification to constrain the rigid-body flight states. Without loss of
% generality, assume that the vehicle heading is due north.
Vf = airSpeed;
gamma = climbAngle;
beta = sideSlip;
% turnRate = trimData.turnRate;
psi = 0;
% In the first instance, check the trim without any turnRate;
p = 0;
q = 0;
r = 0;
% We have now assigned values to 6 of the nine states used in the platform
% equations of motion. What remains are the translational velocity terms.
% 
xed = Vf*cos(gamma)*cos(beta+psi);
yed = Vf*cos(gamma)*sin(beta+psi);
zed = Vf*sin(gamma);
% Euler matrix
Cpsi = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0;0 0 1];
Ctht = [cos(tht) 0 -sin(tht);0 1 0; sin(tht) 0 cos(tht)];
Cphi = [1 0 0;0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
uv = Cphi*Ctht*Cpsi*[xed;yed;zed];
u = uv(1,1);
v = uv(2,1);
w = uv(3,1);
% To ensure a clean trim, the actuator dynamics need to be included.
% Simplest way is to estimate the steady state values.
KT = trimData.paramStruct.KT;
KQ = trimData.paramStruct.KQ;
T1 = KT*omega(1,1); T2 = KT*omega(2,1); 
T3 = KT*omega(3,1); T4 = KT*omega(4,1); 
Q1 = KQ*omega(1,1); Q2 = KQ*omega(2,1); 
Q3 = KQ*omega(3,1); Q4 = KQ*omega(4,1);
% Now re-pack the state vector;
xtrim = [u v w p q r phi tht psi 0 0 0 T1 T2 T3 T4 Q1 Q2 Q3 Q4]';
utrim = omega;
end
