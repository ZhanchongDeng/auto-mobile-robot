function [gridCentersX, gridCentersY, gridSizeX, gridSizeY] = calculateGridCentersFromGridCorners(gridX, gridY, n, m, xlim, ylim)
% CALCULATEGRIDCENTERSFROMGRIDCORNERS: given the gird locations at bottom
% right corner, calculate the locations of grid centers
%
% INPUTS
%   gridX:  NxM positions of x
%   gridY:  NxM positions of y
%   n:      num of rows of the grid
%   m:      num of cols of the grid
%   xlim:   x-limit of the gridmap [xlower, xupper]
%   ylim:   y-limit of the gridmap [ylower, yupper] 
%   
% OUTPUTS
%   gridCenterX, gridCenterY: NxM position of the grid center
%
% Author: Meng, Qian
%
    gridSizeX = (xlim(2) - xlim(1)) / m;
    gridSizeY = (ylim(2) - ylim(1)) / n;
    gridCentersX = gridX + gridSizeX / 2;
    gridCentersY = gridY + gridSizeY / 2;
end