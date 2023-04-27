function [in] = line_in_obstacle(x1, y1, x2, y2, obstacle)
% LINE_IN_OBSTACLE: check if the line is in obstacle
%   INPUTS:
%       [x1 y1 x2 y2]   line to check 
%       obstacle        k-by-2 vertices [x,y] of obstacle in
%                       counterclockwise order
%   OUTPUTS:
%       in              true if the line is in obstacle (not including the
%                       case where line is an edge of the obstacle
    %% Check if line is an edge of the obstacle
    if is_edge([x1, y1, x2, y2], obstacle)
        in = false;
    else
        num_pts = 500;
        check_pts = zeros(num_pts, 2); 
        check_pts(:, 1) = linspace(x1, x2, num_pts)';
        check_pts(:, 2) = linspace(y1, y2, num_pts)';
        is_inside = inpolygon(check_pts(2:end-1, 1), check_pts(2:end-1, 2), obstacle(:, 1), obstacle(:, 2));
        in = any(is_inside);
    end
end