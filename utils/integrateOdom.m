function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
assert(size(initPose,1) == 3 && size(initPose,2) == 1, 'initPose must be a 3-by-1 vector');
% initialize empty final pose 3 by N
finalPose = zeros(3,length(d) + 1);
% set the initial pose
finalPose(:,1) = initPose;
% loop over d and phi
for i = 1:length(d)
    if phi(i) == 0
        % if phi is 0, then the robot is moving straight
        finalPose(1, i+1) = finalPose(1, i) + d(i) * cos(finalPose(3, i));
        finalPose(2, i+1) = finalPose(2, i) + d(i) * sin(finalPose(3, i));
        finalPose(3, i+1) = finalPose(3, i);
    else
        % calculate the radius of curvature
        r = d(i) / phi(i);
        % calculate angle in world frame
        theta = finalPose(3, i) + phi(i);
        % calculate the change in x and y
        dx = r * (sin(theta) - sin(finalPose(3, i)));
        dy = r * (cos(finalPose(3, i)) - cos(theta));
        finalPose(1, i+1) = finalPose(1, i) + dx;
        finalPose(2, i+1) = finalPose(2, i) + dy;
%         finalPose(3, i+1) = theta - 2*pi*floor( (theta+pi)/(2*pi));
        finalPose(3, i+1) = theta;
    end
end
% remove the first column
finalPose = finalPose(:,2:end);
end