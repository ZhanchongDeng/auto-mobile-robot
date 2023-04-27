function [is_free] = check_pt_free(pt, obs)
% CHECK_PT_FREE: return true if the pt is in Qfree
%   INPUTS:
%       [x y]       2D pt
%       obs         1-by-n cell array of n obstacles, each obstacle is k-by-2
%
%   OUTPUTS:
%       is_free     bool true if [x y] is in Qfree
    is_free = true;
    x = pt(1);
    y = pt(2);
    for i=1:size(obs, 2)
        obstacle = obs{i};
        if inpolygon(x, y, obstacle(:,1), obstacle(:,2))
            is_free = false;
            return;
        end
    end
end