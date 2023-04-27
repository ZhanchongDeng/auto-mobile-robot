function [is_free] = check_rectangle_free(x1, y1, x2, y2, obs, r)
% CHECK_RECTANGLE_FREE: given a line and obstacles, check if the rectangle 
% surrounding the line with width 2r is in free space
%
%   INPUTS:
%       [x1 y1 x2 y2]   line to check 
%       obs             1-by-n cell array of n obstacles, each obstacle is k-by-2
%       r               radius of the circular robot
%
%   OUTPUTS:
%       is_free     bool indicating whether rec is in free space
    [l1, l2] = build_rectangle_line(x1, y1, x2, y2, r);
    is_free = check_free(x1, y1, x2, y2, obs) && ...
              check_free(l1(1), l1(2), l1(3), l1(4), obs) && ...
              check_free(l2(1), l2(2), l2(3), l2(4), obs);
%     is_free = true;
%     num_obs = size(obs, 2);
%     for i=1:num_obs
%         obstacle = obs{i};
%         polyout = intersect(obstacle, rec)
%         if any(intersect(obstacle, rec))
%             is_free = false;
%             return;
%         end
%     end
end