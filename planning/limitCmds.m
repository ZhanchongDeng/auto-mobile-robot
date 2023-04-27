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
    vl = fwdVel - (angVel * wheel2Center);
    vr = fwdVel + (angVel * wheel2Center);
    if abs(vl) > maxV
        factor = abs(vl / maxV);
        vl = vl / factor;
        vr = vr / factor;
    end
    if abs(vr) > maxV
        factor = abs(vr / maxV);
        vl = vl / factor;
        vr = vr / factor;
    end
    cmdV = 1/2 * (vl + vr);
    cmdW = 1/(2*wheel2Center) * (vr - vl);
end
