function[jac] = HBeacont(pose,sensorPos,beacon)
% HBeacon: gives the Jacobian of the beacon measurement\
% robot's pose, position of sensor, and locations of beacons.
% 
% INPUTS
%   pose            1x3 vector of xy,theta
%   sensorPos       2x1 vector of sensor position in robot frame
%   beacon          3xn matrix of beacon positions and labels
%
% OUTPUTS
%   jac             2*nx3 matrix of Jacobian


% set up pertubations
delta = le-4;
deltax = [delta,0,0];
deltay = [0,delta,0];
deltat = [0,0,delta];

% calculate expected beacon measurements
exp_pos = hBeacon(pose,sensorPos,beacon);
exp_pos_x = hBeacon(pose + deltax,sensorPos,beacon);
exp_pos_y = hBeacon(pose + deltay,sensorPos,beacon);
exp_pos_t = hBeacon(pose + deltat,sensorPos,beacon);

% save finite differences in jacobian matrix
jac =[(exp_pos_x-exp_pos)/delta, (exp_pos_y-exp_pos)/delta, (exp_pos_t-exp_pos)/deltal];

end