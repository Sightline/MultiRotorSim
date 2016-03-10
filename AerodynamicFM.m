function [X,Y,Z,L,M,N] = AerodynamicFM(x,u,w,surfacePosn,surfaceDCM,nSurf,type,level)
%AERODYNAMICFM A function to calculate the aerodynamic forces and moments
%acting on the aircraft.
%   This function uses the states (x), controls (u), external disturbances
%   (w), aerodynamic surface position (surfacePos) and direction cosine 
%   matrix (surfaceDCM) and returns the total body-axis aerodynamic forces 
%   and moments (X,Y,Z,L,M,N).
%
%   Author: Dr David Anderson
%   Date:   12/10/2015
%
%   Input variables:    x   -   current system state vector
%                       u   -   current control vector
%                       w   -   vector of exogeneous disturbances
%                       surfacePos -  position of the aerodynamics surface 
%                                     c.p. in body axes.
%                       surfaceDCM -  aerodynamics surface direction cosine 
%                                     matrix wrt body axes.
%                       nProp    -  number of propulsion units
%
%   Output variables:   X   -  Sum of all aerodynamic forces acting along
%                              the body x-axis.
%                       Y   -  Sum of all aerodynamic forces acting along
%                              the body y-axis.
%                       Z   -  Sum of all aerodynamic forces acting along
%                              the body z-axis.
%                       L   -  Sum of all aerodynamic moments acting around
%                              the body x-axis.
%                       M   -  Sum of all aerodynamic moments acting around
%                              the body y-axis.
%                       N   -  Sum of all aerodynamic moments acting around
%                              the body z-axis.
%
%   Local variables:    forceVector  -  vector used for force summation
%                       momentVector -  vector used for moment summation
%% Initialise variables
X = 0; Y = 0; Z = 0;        % Appropriate return values if no propulsion
L = 0; M = 0; N = 0;        % units are present.
if(isempty(nSurf))
    return;
end
% Initialise the container arrays
forceVector = [0;0;0]; momentVector = [0;0;0];
%% Calculate aerodynamic unit FM
% In this section, the forces and moments produced by each aerodynamic
% surface are computed in the native axes set.
%
switch type
    case 'FixedWing'
        % Not implemented at present
    case 'Hybrid'
        % Not implemented at present
    case 'MultiRotor'
        % For the multirotor, we can estimate the drag experienced by the
        % rotor disc (H-Force in the helicopter literature), the pitching
        % moment acting on the disc and also the fuselage drag.
        %
        % Fuselage drag component
        Vf2 = x(1,1)^2+x(2,1)^2+x(3,1)^2;
        if(Vf2 == 0)
            Vf2 = 0.0001;
        end
        Cdf = 0.01; 
        D = Cdf * Vf2;
        %
        alpha = atan2(x(3,1),x(1,1));
        beta = asin(x(2,1)/sqrt(Vf2));
        T1 = [cos(beta) -sin(beta) 0;sin(beta) cos(beta) 0;0 0 1];
        T2 = [cos(alpha) 0 -sin(alpha);0 1 0;sin(alpha) 0 cos(alpha)];
        forceVector = forceVector + T2*T1*[-D;0;0];       
end
%% Now return the propulsive forces and moments
%
X = forceVector(1,1);
Y = forceVector(2,1);
Z = forceVector(3,1);
L = momentVector(1,1);
M = momentVector(2,1);
N = momentVector(3,1);

end

