function rd = Guidance(x,du)
%GUIDANCE The guidance function generates a feasible commanded velocity
%vector that the control module has to achieve.
%   This function uses the states (x) and control sampling interval (du) to
%   generate a commanded velocity vector for the control module. The
%   guidance module has two functions. First, a desired path is generated.
%   This can be a simple 4D waypoint-defined piecewise continuous path
%   composed of linear segments, or it can be a polynomial of defined
%   degree, one suitable for capturing the curvature and inflection points
%   necessary to define the manoeuvre. The second phase is to take this
%   path and construct a set of commanded velocity vector components that
%   are feasible for the system platform simulated.
%
%   Author: Dr David Anderson
%   Date:   12/10/2015
%
%   Input variables:    x   -   current system state vector
%                       du  -   control sampling interval
%
%   Output variables:   rd  -   Structure containing the commanded velocity
%                               vector in the appropriate form.
%
%% Define the desired trajectory
% Waypoint guidance.
% Define the waypoints
clear; clc;
nwp = 5; du = 0.01;
p = cell(1,nwp);
p{1} = [0 0 0 0];
p{2} = [10 0 -2 5];
p{3} = [20 10 -2 10];
p{4} = [10 20 -2 15];
p{5} = [0 0 0 20];
% Construct the commands
numPoints = ceil(20/du);
lb = 1; t = 0;
for ii=1:numPoints
    % find the bounding waypoints. 
    if(~(t<p{lb+1}(4)))
        % increase lower bound
        lb = lb+1;
    end
    % Interpolate between waypoints.
    % Calculate lambda.
    lambda = (t-p{lb}(4))/(p{lb+1}(4)-p{lb}(4));
    % Now interpolate the waypoints
    rd.x(ii) = (lambda)*p{lb+1}(1) + (1-lambda)*p{lb}(1);
    rd.y(ii) = (lambda)*p{lb+1}(2) + (1-lambda)*p{lb}(2);
    rd.z(ii) = (lambda)*p{lb+1}(3) + (1-lambda)*p{lb}(3);
    rd.t(ii) = (lambda)*p{lb+1}(4) + (1-lambda)*p{lb}(4);
    rd.xd(ii) = (p{lb+1}(1)-p{lb}(1))/(p{lb+1}(4)-p{lb}(4));
    rd.yd(ii) = (p{lb+1}(2)-p{lb}(2))/(p{lb+1}(4)-p{lb}(4));
    rd.zd(ii) = (p{lb+1}(3)-p{lb}(3))/(p{lb+1}(4)-p{lb}(4));
    rd.xdd(ii) = 0;
    rd.xdd(ii) = 0;
    rd.xdd(ii) = 0;
    t = t+du;
end
% Polynomial trajectory

rd.zdem = -1;

%% Construct feasible command
% -------- Differential flatness method ------------
%


end

