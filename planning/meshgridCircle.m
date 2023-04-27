function [points] = meshgridCircle(bound)
%   INPUTS:
%       bound       1 x 3 array [centerx, centery, radius] of bound
%   OUTPUTS
%       points      k x 2 array of points in the bound
    centerx = bound(1);
    centery = bound(2);
    radius = bound(3);
    n = 50;
    m = 50;
    xlim = [centerx - radius, centerx + radius];
    ylim = [centery - radius, centery + radius];
    [gridX, gridY] = meshgrid(linspace(xlim(1), xlim(2), n), linspace(ylim(1), ylim(2), m));
    [gridCentersX, gridCentersY, ~, ~] = calculateGridCentersFromGridCorners(gridX, gridY, n, m, xlim, ylim);
    gridCentersX = gridCentersX(:);
    gridCentersY = gridCentersY(:);
    num_pts = size(gridCentersX, 1);
    points = zeros(num_pts, 2);
    points(:, 1) = gridCentersX;
    points(:, 2) = gridCentersY;
    distances = zeros(num_pts, 1);
    for i=1:num_pts
        distances(i) = norm(points(i, :) - bound(1:2));
    end
    points = points(distances > radius, :);
    if mod(size(points, 1),2) ~= 0
        points = points(1:end-1, :);
    end
end