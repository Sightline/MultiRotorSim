function omega = Controller(x,rd,du)
%CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
persistent intVec;
if(isempty(intVec)) 
    intVec = 0;
end
zdem = rd.zdem;
z = x(12,1);
w = x(3,1);
posErr = zdem - z;
Kp = 2; Ki = 0.7;
posLoopIntControl = intVec + Ki*du*posErr;
intVec = posLoopIntControl;
posLoopControl = Kp*posErr + intVec + posLoopIntControl;
rateErr = posLoopControl - w;
Kr = -50;
omega_sc = Kr*rateErr;
if (omega_sc < 0)
    omega_sc = 0;
end
if (omega_sc > 100)
    omega_sc = 100;
end
omega = [1;1;1;1]*omega_sc;
end

