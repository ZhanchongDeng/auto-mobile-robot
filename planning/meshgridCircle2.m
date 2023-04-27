function [pointsX, pointsY] = meshgridCircle2(bound, n, m)
%   INPUTS:
%       bound       1 x 3 array [centerx, centery, radius] of bound
%   OUTPUTS
%       points      k x 2 array of points in the bound
    centerx = bound(1);
    centery = bound(2);
    radius = bound(3);
    if nargin <= 1
        n = 100;
        m = 100;
    end
    xlim = [centerx - radius, centerx + radius];
    ylim = [centery - radius, centery + radius];
    [pointsX, pointsY] = meshgrid(linspace(xlim(1), xlim(2), n), linspace(ylim(1), ylim(2), m));
end