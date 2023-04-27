function [hmap] = plotSphereWorld(sphereworld, does_hold_off, points)
% PLOTSPHEREWORLD: given a sphere world map in txt file, plot the map and
% return the handler
%
% INPUTS
%   sphereworld_txtfile: a txt file where the Column 1 and 2 contain the X 
%   and Y coordinates of the centers of the spheres, and column 3 contain 
%   the radius.
%   The first line describes the boundary of the workspace while all other 
%   lines describe the obstacles.
%   does_hold_off: bool indicating whether to hold off at the end
%
% RETURNS
%   hmap: handler of the plot
if nargin <= 2
    does_hold_off = true;
end
% sphereworld = importdata(sphereworld_txtfile);
% disp(size(sphereworld, 1))
hmap = figure();
pboundary = circle(sphereworld(1, 1:2), sphereworld(1, 3), 1000);
hold on
for i=2:size(sphereworld, 1)
    pobstacle = circle(sphereworld(i, 1:2), sphereworld(i, 3), 1000, 'r-');
    hold on
end
if nargin >=3
    ppts = scatter(points(:, 1), points(:, 2), '.g');
end
xlabel('x (m)')
ylabel('y (m)')
if nargin >=3
    legend([pboundary, pobstacle, ppts], ...
        'Boundary', 'Obstacles', 'Points')
else
    legend([pboundary, pobstacle], ...
        'Boundary', 'Obstacles')
end
title('Sphere World')
if does_hold_off
    hold off
end