function [X,Y,Z,L,M,N] = PropulsiveFM(x,u,w,unitPosn,unitDCM,nProp,type,level)
%PROPULSIVEFM A function to calculate the propulsive forces and moments
%acting on the aircraft.
%   This function uses the states (x), controls (u), external disturbances
%   (w), propulsive unit position (unitPos) and direction cosine matrix
%   (unitDCM) and returns the total body-axis propulsive forces and moments
%   (X,Y,Z,L,M,N).
%
%   Author: Dr David Anderson
%   Date:   12/10/2015
%
%   Input variables:    x   -   current system state vector
%                       u   -   current control vector
%                       w   -   vector of exogeneous disturbances
%                       unitPosn -  position of the propulsion unit in body
%                                   axes.
%                       unitDCM  -  propoltion unit direction cosine matrix
%                                   wrt body axes.
%                       nProp    -  number of propulsion units
%
%   Output variables:   X   -   Sum of all propulsive forces acting along
%                               the body x-axis.
%                       Y   -   Sum of all propulsive forces acting along
%                               the body y-axis.
%                       Z   -   Sum of all propulsive forces acting along
%                               the body z-axis.
%                       L   -   Sum of all propulsive moments acting around
%                               the body x-axis.
%                       M   -   Sum of all propulsive moments acting around
%                               the body y-axis.
%                       N   -   Sum of all propulsive moments acting around
%                               the body z-axis.
%
%   Local variables:    forceVector  -  vector used for force summation
%                       momentVector -  vector used for moment summation
%% Initialise variables
X = 0; Y = 0; Z = 0;        % Appropriate return values if no propulsion
L = 0; M = 0; N = 0;        % units are present.
if(isempty(nProp))
    return;
end
% Initialise the container arrays
forceVector = [0;0;0]; momentVector = [0;0;0];
%% Calculate propulsive unit FM
% In this section, the forces and moments produced by each propulsion unit
% are computed in the native axes set.
%
switch type
    case 'MultiRotor'
        switch level
            case 'L1'
                % define the rotor directions. 
                rotDir = [1,-1,1,-1];
                % loop over all propulsive units
                for ii = 1:nProp
                    % For each propulsion unit, calculate the force in 
                    % local axes. For the L1 multirotor, propulsive force 
                    % (thrust) is part of the state vector x.
                    thrust = [0;0;-x(12+ii,1)];
                    thrustBodyAxes = unitDCM{ii}'*thrust;
                    momentBodyAxes = cross(unitPosn{ii},thrustBodyAxes)...
                                   + x(16+ii,1)*rotDir(ii);
                    forceVector = forceVector + thrustBodyAxes;
                    momentVector = momentVector + momentBodyAxes;
                end
            case 'L2'
                % Not implemented at present
            case 'L3'
                % Not implemented at present
        end
    case 'FixedWing'
        % Not implemented at present
    case 'Hybrid'
        % Not implemented at present
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
