function [flag] = is_edge(line, obstacle)
% IS_EDGE: check if the line is an edge of the obstacle
%   INPUTS:
%       line            [x1 y1 x2 y2] line to check 
%       obstacle        k-by-2 vertices [x,y] of obstacle in
%                       counterclockwise order
%   OUTPUTS:
%       flag            true if the line is an edge of the obstacle
%
    flag = false;
    for i=1:size(obstacle, 1)-1
        edge = horzcat(obstacle(i, :), obstacle(i+1, :));
        if isequal(line, edge)
            flag = true;
            return;
        end
    end
end