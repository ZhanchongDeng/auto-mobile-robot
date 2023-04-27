function [is_free] = check_circle_free(pt, obs, radius)
% CHECK_PT_FREE: return true if the pt is in Qfree
%   INPUTS:
%       [x y]       2D pt
%       obs         1-by-n cell array of n obstacles, each obstacle is k-by-2
%       radius      scalar, radius of the robot
%
%   OUTPUTS:
%       is_free     bool true if [x y] is in Qfree
    is_free = true;
    pts = meshgridCircle([pt(1), pt(2), radius]);
    xs = pts(:, 1);
    ys = pts(:, 2);
    for i=1:size(obs, 2)
        obstacle = obs{i};
        is_inside = inpolygon(xs, ys, obstacle(:,1), obstacle(:,2));
        if any(is_inside)
            is_free = false;
            return;
        end
    end
end