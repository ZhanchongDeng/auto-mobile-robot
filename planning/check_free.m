function [is_free] = check_free(x1, y1, x2, y2, obs)
% CHECK_FREE: given a line and obstacles, check if the line is in free
% space
%
%   INPUTS:
%       [x1 y1 x2 y2]   line to check 
%       obs             1-by-n cell array of n obstacles, each obstacle is k-by-2
%
%   OUTPUTS:
%       is_free     bool indicating whether line is in free space
    is_free = true;
    num_obs = size(obs, 2);
    for i=1:num_obs
        obstacle = obs{i};
        if line_in_obstacle(x1, y1, x2, y2, obstacle)
            is_free = false;
            return;
        end
        for j=1:size(obstacle, 1)
            if j ~= size(obstacle, 1)
                k = j + 1;
            else
                k = 1;
            end
            x3 = obstacle(j, 1);
            y3 = obstacle(j, 2);
            x4 = obstacle(k, 1);
            y4 = obstacle(k, 2);
            [isect,x,y] = intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
            if isect && (x ~= x1 && y ~= y1) && (x ~= x2 && y ~= y2)
                is_free = false;
                return;
            end
        end
    end
end