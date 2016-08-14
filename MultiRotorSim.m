%%-------------------------------------------------------------------------
%   MultiRotor Simulation: A simulation model for examining the flight
%   mechanics of multirotor micro-UAVs.
%
%   Author: Dr David Anderson
%   Date: 3/10/2015
%--------------------------------------------------------------------------
%   Revisions:
%
%
%
%--------------------------------------------------------------------------
clc;clear;
%% Define simulation parameters
nstates = 20;
ncontrols = 4;
trimFlag = true;
trimSim = false;
lineariseFlag = true;
%% Define constants
g = 9.81;               % gravitational acceleration (m/s^2)
% Inertial properties
m = 1.4;                % mass (kg)
Ixx = 0.1;              % roll axis inertia (kg/m^2)
Iyy = 0.1;              % pitch axis inertia (kg/m^2)
Izz = 0.1;              % yaw axis inertia (kg/m^2)
% Aerodynamic surface properties
numSurfaces = 0;
surfacePosn = cell(numSurfaces,1);
surfaceDCM = cell(numSurfaces,1);
% Rotor properties
numRotors = 4;          % number of rotors
numLinks = 1;           % number of fixed linkages per rotor assembly
KT = 0.1;               % Rotor thrust gain (Ns/m)
KQ = 0.05;              % Rotor torque gain (Ns)
tauT = 0.1;             % Rotor thrust time constant
tauQ = 0.1;             % Rotor torque time constant
% in the following, the notation is {rotor i, link j}.
rotorLinks{1,1} = [0.1 0 0]';
rotorLinkAngles{1,1} = [0 0 -45.0*(pi/180.0)];
rotorLinks{2,1} = [0.1 0 0]';
rotorLinkAngles{2,1} = [0 0 45.0*(pi/180.0)];
rotorLinks{3,1} = [0.1 0 0]';
rotorLinkAngles{3,1} = [0 0 135.0*(pi/180.0)];
rotorLinks{4,1} = [0.1 0 0]';
rotorLinkAngles{4,1} = [0 0 -135.0*(pi/180.0)];
% now calculate the rotor position wrt cg in body axes.
rotorPosn = cell(numRotors,1);
rotorDCM = cell(numRotors,1);
for ii = 1:numRotors
    dummyPos = zeros(3,1);
    for jj = 1:numLinks
        % We need to calculate each link position and nominal attitude wrt
        % body axes.
        lphi = rotorLinkAngles{ii,jj}(1);
        ltht = rotorLinkAngles{ii,jj}(2);
        lpsi = rotorLinkAngles{ii,jj}(3);
        Cy = [cos(lpsi) sin(lpsi) 0;-sin(lpsi) cos(lpsi) 0;0 0 1];
        Cp = [cos(ltht) 0 -sin(ltht);0 1 0;sin(ltht) 0 cos(ltht)];
        Cr = [1 0 0;0 cos(lphi) sin(lphi);0 -sin(lphi) cos(lphi)];
        C = Cr*Cp*Cy;
        dummyPos = dummyPos + C'*rotorLinks{ii,jj};
    end
    rotorPosn{ii} = dummyPos;
    rotorDCM{ii} = C;
end
%
% Now package all of the aircraft parametric data into a single structure
paramStruct.g = g;
paramStruct.m = m;
paramStruct.I = [Ixx 0 0;0 Iyy 0;0 0 Izz];
paramStruct.numRotors = numRotors;
paramStruct.rotorPosn = rotorPosn;
paramStruct.rotorDCM = rotorDCM;
paramStruct.numSurfaces = numSurfaces;
paramStruct.surfacePosn = surfacePosn;
paramStruct.surfaceDCM = surfaceDCM;
paramStruct.KT = KT;
paramStruct.KQ = KQ;
paramStruct.tauT = tauT;
paramStruct.tauQ = tauQ;

%% Trim model
% The aircraft needs to be placed in trim prior to each run for valid
% results to be obtained in a form required by linear analysis.
X0 = zeros(nstates,1); U0 = zeros(ncontrols,1);
if(trimFlag)
    [xtrim,utrim] = myTrim(X0,U0,0,0,0,0,paramStruct);
end

%% Linearise model
% It is often useful to linearise the model to view the stability
% derivatives of the aircraft and the poles/zeros of the dynamic motion.
if(lineariseFlag)
    [sysMatrix,conMatrix] = lineariseModel(xtrim,utrim,paramStruct);
end

%% Time response simulation
%
dt = 0.001;                 % Maximum simulation timestep
tend = 10;                  % Simulation final time
du = 0.01;                  % Controller sampling interval
x = xtrim; u = utrim;
nPts = ceil(tend / dt);
stateVector = zeros(nPts,nstates);
controlVector = zeros(nPts,ncontrols);
timeVector = zeros(nPts,1);
%
t = 0; ii = 0;
inttype = 'RK4';
while (t < tend)
    % Now prepare the equations of motion. First, call the guidance &
    % control functions.
    if(~trimSim)
        if(mod(ii,floor(du/dt)) == 0)
            rd = Guidance(x,du);
            u = Controller(x,rd,du);
        end
    end
    % Now integrate the dynamic terms
    switch inttype
        case 'Euler'
            %**********************************************************************
            % Euler Integration
            % This is the simplest integration scheme requiring only one
            % function call to the calculateDerivatives function.
            % Calculate the state derivative
            xd = CalculateDerivatives(x,u,paramStruct);
            % and integrate the state
            x = x + xd*dt;
            % Apply primitive ground constraint
            if(x(12,1) > 0)
                x(12,1) = 0;
            end
        case 'RK4'
            %**********************************************************************
            % 4th ORDER RUNGE-KUTTA
            % The 4th order Runge-Kutta integration algorithm is considered to
            % be sufficiently accurate for dynamic simulation of most aerospace
            % vehicles. It uses 4 evaluations of the derivative function per
            % time-step update, yielding integration errors of the order dt^5.
            % First RK data point
            k_1 = CalculateDerivatives(x,u,paramStruct);
            % Move the current trajectory estimate to the mid-point of the
            % integration step.
            xn = x+0.5*dt*k_1;
            % STEP 2...
            % Second RK data point
            k_2 = CalculateDerivatives(xn,u,paramStruct);
            xn = x+0.5*dt*k_2;
            % STEP 3...
            % Third RK data point
            k_3 = CalculateDerivatives(xn,u,paramStruct);
            xn = x+dt*k_3;
            % STEP 4...
            % Final RK data point
            k_4 = CalculateDerivatives(xn,u,paramStruct);
            % Now use the RK integration equation to combine all of the
            % derivative evaluations
            x = x + (1/6)*(k_1+2*k_2+2*k_3+k_4)*dt;  % main equation
            %
            % Apply primitive ground constraint
            if(x(12,1) > 0)
                x(12,1) = 0;
            end
    end
    %
    % Now update the global time and counter
    %
    t=t+dt;
    ii = ii+1;
    %
    % Finally, write the simulation data to storage vectors for later
    % analysis
    %
    stateVector(ii,:) = x';
    controlVector(ii,:) = u';
    timeVector(ii,1) = t;
end
%
% Print the system and control matrices to screen
%
if(lineariseFlag)
    disp(sysMatrix);
    disp(conMatrix);
end
%
%% Display results
% Rigid-body states
d2r = pi/180;
figure(1);
subplot(4,3,1)
plot(timeVector,stateVector(:,1)); ylabel('u (m/s)');
subplot(4,3,2)
plot(timeVector,stateVector(:,2)); ylabel('v (m/s)');
subplot(4,3,3)
plot(timeVector,stateVector(:,3)); ylabel('w (m/s)');
subplot(4,3,4)
plot(timeVector,stateVector(:,4)); ylabel('p (rad/s)');
subplot(4,3,5)
plot(timeVector,stateVector(:,5)); ylabel('q (rad/s)');
subplot(4,3,6)
plot(timeVector,stateVector(:,6)); ylabel('r (rad/s)');
subplot(4,3,7)
plot(timeVector,stateVector(:,7)./d2r); ylabel('\phi (deg)');
subplot(4,3,8)
plot(timeVector,stateVector(:,8)./d2r); ylabel('\theta (deg)');
subplot(4,3,9)
plot(timeVector,stateVector(:,9)./d2r); ylabel('\psi (deg)');
subplot(4,3,10)
plot(timeVector,stateVector(:,10)); ylabel('xe (m)'); xlabel('t(s)');
subplot(4,3,11)
plot(timeVector,stateVector(:,11)); ylabel('ye (m)'); xlabel('t(s)');
subplot(4,3,12)
plot(timeVector,stateVector(:,12)); ylabel('ze (m)'); xlabel('t(s)');
axes('Units','Normal');
h = title('MultiRotor simulation time response diagrams - States');
set(gca,'visible','off')
set(h,'visible','on')
%
% Rotor states
%
figure(2);
subplot(2,4,1)
plot(timeVector,stateVector(:,13)); ylabel('T (N)'); xlabel('t(s)');
subplot(2,4,2)
plot(timeVector,stateVector(:,14)); ylabel('T (N)'); xlabel('t(s)');
subplot(2,4,3)
plot(timeVector,stateVector(:,15)); ylabel('T (N)'); xlabel('t(s)');
subplot(2,4,4)
plot(timeVector,stateVector(:,16)); ylabel('T (N)'); xlabel('t(s)');
subplot(2,4,5)
plot(timeVector,stateVector(:,17)); ylabel('Q1 (Nm)'); xlabel('t(s)');
subplot(2,4,6)
plot(timeVector,stateVector(:,18)); ylabel('Q2 (Nm)'); xlabel('t(s)');
subplot(2,4,7)
plot(timeVector,stateVector(:,19)); ylabel('Q3 (Nm)'); xlabel('t(s)');
subplot(2,4,8)
plot(timeVector,stateVector(:,20)); ylabel('Q4 (Nm)'); xlabel('t(s)');
axes('Units','Normal');
h = title('MultiRotor simulation time response diagrams - Rotor FM');
set(gca,'visible','off')
set(h,'visible','on')
%
% Controls
%
figure(3);
subplot(2,2,1)
plot(timeVector,controlVector(:,1)); ylabel('\omega_1 (rad/s)'); xlabel('t(s)');
subplot(2,2,2)
plot(timeVector,controlVector(:,2)); ylabel('\omega_2 (rad/s)'); xlabel('t(s)');
subplot(2,2,3)
plot(timeVector,controlVector(:,3)); ylabel('\omega_3 (rad/s)'); xlabel('t(s)');
subplot(2,2,4)
plot(timeVector,controlVector(:,4)); ylabel('\omega_4 (rad/s)'); xlabel('t(s)');
axes('Units','Normal');
h = title('MultiRotor simulation time response diagrams - Rotor FM');
set(gca,'visible','off')
set(h,'visible','on')
%
% 3D plot
%
figure(4);
plot3(stateVector(:,11),stateVector(:,10),-stateVector(:,12));
grid on;
ylabel('North (m)'); xlabel('East (m)'); zlabel('Altitude (m)');
%% CleanUp and end simulation