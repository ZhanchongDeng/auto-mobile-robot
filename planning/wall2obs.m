function [obs] = wall2obs(wall, r)
% WALL2OBS: Given wall map, return obstacles
%
%   INPUTS:
%       wall    k-by-4 lines [x1 y1 x2 y2]
%       r       robot radius
%   OUTPUTS:
%       obs     cell array of obstacles, each ob m-by-2 matrix
%
    num_walls = size(wall, 1);
    obs = cell(1,num_walls);
    for i=1:num_walls
        obs{i} = build_rectangle(wall(i, 1), wall(i, 2), wall(i, 3), wall(i, 4), r);
    end
end