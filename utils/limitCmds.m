function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   LASTNAME, FIRSTNAME 

% find vL and vR from V and w
leftWVel = fwdVel - angVel * wheel2Center;
rightWVel = fwdVel + angVel * wheel2Center;

% scale both by maximum velocity
if abs(leftWVel) > maxV || abs(rightWVel) > maxV
    if abs(leftWVel) > abs(rightWVel)
        ratio = maxV / abs(leftWVel);
    else
        ratio = maxV / abs(rightWVel);
    end
    
    rightWVel = rightWVel * ratio;
    leftWVel = leftWVel * ratio;
end

% find new V and w from scaled vL and vR
cmdV = (leftWVel + rightWVel) / 2;
cmdW = (rightWVel - leftWVel) / (2*wheel2Center);

end