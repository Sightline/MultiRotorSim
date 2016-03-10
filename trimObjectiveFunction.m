function cost = trimObjectiveFunction(trimVariables,trimData)
%TRIMOBJECTIVEFUNCTION Function to compute the cost function used in the
%trim minimisation algorithm
%   Detailed explanation goes here
%
%
%
%   Author: Dr David Anderson
%   Date:   12/10/2015
%
%
%% Unpack the trim & control data
% First the trim data
Vf = trimData.Vf;
gamma = trimData.gamma;
beta = trimData.beta;
% turnRate = trimData.turnRate;
KT = trimData.paramStruct.KT;
KQ = trimData.paramStruct.KQ;
% Now the control data
omega(1,1) = trimVariables(1);
omega(2,1) = trimVariables(2);
omega(3,1) = trimVariables(3);
omega(4,1) = trimVariables(4);
phi = trimVariables(5);
tht = trimVariables(6);
%% Constrain the states
% Here we need to use all the information contained within the trim state
% specification to constrain the rigid-body flight states. Without loss of
% generality, assume that the vehicle heading is due north.
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
%
% Select trim method
%
trimMethod = 'simple';
switch trimMethod
    case 'simple'
        % Calculate the cost function - Simple method
        % To ensure a clean trim, the actuator dynamics need to be included
        % Simplest way is to estimate the steady state values.
        T1 = KT*omega(1,1); T2 = KT*omega(2,1);
        T3 = KT*omega(3,1); T4 = KT*omega(4,1);
        Q1 = KQ*omega(1,1); Q2 = KQ*omega(2,1);
        Q3 = KQ*omega(3,1); Q4 = KQ*omega(4,1);
        % Now re-pack the state vector;
        x = [u v w p q r phi tht psi 0 0 0 T1 T2 T3 T4 Q1 Q2 Q3 Q4]';
        %
        % Next step in the process is to calculate the derivatives.
        xd = CalculateDerivatives(x,omega,trimData.paramStruct);
        % The cost function is the sum-of-squares of the six body-axes
        % accelerations.
        Q = diag([1 1 1 1 1 1]);
        xdr = xd(1:6,1);
        cost = xdr'*Q*xdr;
    case 'integral'
        % Calculate the cost function - integral method
        % In the integral method, we make no assumption about the 
        % higher-order dynamics.
        T1 = 0; T2 = 0;
        T3 = 0; T4 = 0;
        Q1 = 0; Q2 = 0;
        Q3 = 0; Q4 = 0;
        % Now re-pack the state vector;
        x = [u v w p q r phi tht psi 0 0 0 T1 T2 T3 T4 Q1 Q2 Q3 Q4]';
        % With this method, we integrate the state trajectory until the
        % higher-order dynamics have reached steady state and then 
        % calculate the cost function. The integration parameters should be 
        % the same as those selected for the main time-response 
        % calculations.
        %
        dt = 0.001;                 % Maximum simulation timestep
        tend = 1.0;                 % Simulation final time
        %
        t = 0; ii = 0;
        inttype = 'RK4';
        while (t < tend)
            % Integrate the dynamic terms
            switch inttype
                case 'Euler'
                    %******************************************************
                    % Euler Integration
                    % This is the simplest integration scheme requiring 
                    % only one function call to the calculateDerivatives 
                    % function.
                    % Calculate the state derivative
                    xd = CalculateDerivatives(x,omega,trimData.paramStruct);
                    % and integrate the state
                    x = x + xd*dt;
                    % Apply primitive ground constraint
                    if(x(12,1) > 0)
                        x(12,1) = 0;
                    end
                case 'RK4'
                    %******************************************************
                    % 4th ORDER RUNGE-KUTTA
                    % The 4th order Runge-Kutta integration algorithm is 
                    % considered to be sufficiently accurate for dynamic 
                    % simulation of most aerospace vehicles. It uses 4 
                    % evaluations of the derivative function per
                    % time-step update, yielding integration errors of the 
                    % order dt^5.
                    % First RK data point
                    k_1 = CalculateDerivatives(x,omega,trimData.paramStruct);
                    % Move the current trajectory estimate to the mid-point
                    %  of the integration step.
                    xn = x+0.5*dt*k_1;
                    % STEP 2...
                    % Second RK data point
                    k_2 = CalculateDerivatives(xn,omega,trimData.paramStruct);
                    xn = x+0.5*dt*k_2;
                    % STEP 3...
                    % Third RK data point
                    k_3 = CalculateDerivatives(xn,omega,trimData.paramStruct);
                    xn = x+dt*k_3;
                    % STEP 4...
                    % Final RK data point
                    k_4 = CalculateDerivatives(xn,omega,trimData.paramStruct);
                    % Now use the RK integration equation to combine all of 
                    % the derivative evaluations
                    x = x + (1/6)*(k_1+2*k_2+2*k_3+k_4)*dt;  % main eqn
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
        end
        %
        % At this point, calculate the state derivative vector with the 
        % current state and control estimates.
        xd = CalculateDerivatives(x,omega,trimData.paramStruct);
        % The cost function is the sum-of-squares of the six body-axes
        % accelerations.
        Q = diag([1 1 1 1 1 1]);
        xdr = xd(1:6,1);
        cost = xdr'*Q*xdr;
end
end