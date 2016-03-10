function [sysMatrix, conMatrix] = lineariseModel(xtrim, utrim, paramStruct)
%UNTITLED This function is used to numerically linearise the model.
%
%
%   Author: Dr David Anderson
%   Date:   20/10/2015
%
%
%%  Perturb the states and controls
nstates = length(xtrim);
ncontrols = length(utrim);
nv = nstates+ncontrols;
pert = zeros(nv,1); dvar = pert;
stepSize = 0.01;
minPertSize = 0.0001*ones(1,nv);
cv = [xtrim;utrim];
for i=1:nv
    pert(i)=abs(cv(i))*stepSize;
    if pert(i) < minPertSize(i)
        pert(i)=minPertSize(i);
    end
    dvar(i)=2.0*pert(i);
end
%
%
%
var = [xtrim' utrim'];
varpert = zeros(2*nv+1,nv);
for m=1:(2*nv+1)
    for n=1:nv
        varpert(m,n)=var(n);
    end
    if m > 1
        if mod(m,2) == 1
            varpert(m,fix(0.5*m))=var(fix(0.5*m))-pert(fix(0.5*m));
        else
            varpert(m,fix(0.5*m))=var(fix(0.5*m))+pert(fix(0.5*m));
        end
    end
end
%
% evaluate the Jacobian
%
for j=1:(2*nv+1)
    % Calculate the derivative corresponding to the perturbed state.
    xp = varpert(j,1:nstates)';
    up = varpert(j,(nstates+1):nv)';
    xd = CalculateDerivatives(xp,up,paramStruct);
    % Now the function
    for i=1:nstates
        f(i,j) = xd(i); %#ok<AGROW>
    end
end
% calculate the Jacobian using numerical differentiation
for m=1:nstates
    for n=1:nv
        J(m,n)=(f(m,(2*n))-f(m,(2*n+1)))/(dvar(n)); %#ok<AGROW>
    end
end
sysMatrix = J(1:nstates,1:nstates);
conMatrix = J(1:nstates,(nstates+1):nv);
end

