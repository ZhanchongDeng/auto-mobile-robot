function[depth] = depthPredict(robotPose,map, optionalWalls, sensorOrigin,angles)
% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,, OPTIONALWALLS, SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       optionalWalls   M-by-4 matrix containing the coordinages of optional
%                       walls in the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters), output NaN for optional walls
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2

assert(size(robotPose,1) == 3 && size(robotPose,2) == 1, 'robotPose must be a 3-by-1 vector');
assert(size(map,2) == 4, 'map must be an N-by-4 matrix');
assert(size(sensorOrigin,1) == 1 && size(sensorOrigin,2) == 2, 'sensorOrigin must be a 1-by-2 vector');
assert(size(angles,2) == 1, 'angles must be a K-by-1 vector');

depth = zeros(size(angles,1),1);

% add optional walls to regular maps
map = [map; optionalWalls];
optionalIdx = size(map,1) - size(optionalWalls, 1) + 1;

% convert the sensor origin from the robot frame to the global frame

xyGSensor = robot2global(robotPose.', sensorOrigin).';

for i = 1:size(angles, 1)
    min_range = Inf;
    min_depth = Inf;
    % iterate over each wall
    for j = 1:size(map, 1)
        wall = map(j,:);
        % use intersectPoint to find the intersection of the sensor ray with the wall
        big_num = 10000;
        sStartX = xyGSensor(1);
        sStartY = xyGSensor(2);
        sEndX = sStartX + big_num * cos(robotPose(3) + angles(i));
        sEndY = sStartY + big_num * sin(robotPose(3) + angles(i));

        [isect,x,y] = intersectPoint(wall(1), wall(2), wall(3), wall(4), sStartX, sStartY, sEndX, sEndY);
        range = norm([x-xyGSensor(1) y-xyGSensor(2)]);
        if isect
            % if there is an intersection, find the distance to the intersection
            % and update min_depth if necessary
            if range < min_range
                if j >= optionalIdx
                    min_depth = -1;
                    min_range = -1;
                else
                    min_range = range;
                    % depth with respect to the robot frame
                    min_depth = range * cos(angles(i));
                end
            end
        end
    end
    % set the depth to min_depth
    depth(i) = min_depth;

end
depth(depth < 0.175 & depth >= 0) = 0;
depth(depth > 10) = 10;
end