function[exp_pos] = hBeacon(pose,sensorPos,beacon)
% hBeacon: gives the expected beacon measurement based on the robot's pose,
% position of sensor, and locations of beacons .
%
% INPUTS
%   pose:           1x3 vector of x,y,theta
%   sensorPos:      2x1 vector of sensor position in robot frame
%   beacon:         3xn matrix of beacon positions and labels
%
% OUTPUTS
%   exp_pos:        2*nx1 vector of expected beacon measurments


exp_pos = zeros(2*length(beacon),1);

counter = 1;
for i = 1: length(beacon)
    p = global2robot(pose,beacon(i,1:2)) - sensorPos;
    exp_pos(counter) = p(1);
    exp_pos(counter +1)= p(2);
    counter = counter + 2;
end