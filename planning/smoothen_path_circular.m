function [path] = smoothen_path_circular(path, obs, r)
% SMOOTHEN_PATH: Shorten the path by removing ndoes with adjacent nodes in
% Qfree
%
%   INPUTS:
%       path    k-by-2 waypoints [x y]
%       obs     1-by-n cell array of n obstacles, each obstacle is k-by-2
%       r       robot radius
%
%   OUPUTS:
%       path    m-by-2 waypoints, m <= k
    i = 1;
    path_len = size(path, 1);
    while true
%         disp('len')
%         disp(size(path, 1))
%         disp('i')
%         disp(i)
        if i + 2 > size(path, 1)
            i = 1;
            if size(path, 1) == path_len || size(path, 1) <= 2
                break;
            else
                path_len = size(path, 1);
            end
        end
%          disp('len')
%          disp(size(path, 1))
%          disp('i')
%          disp(i)
        if check_rectangle_free(path(i, 1), path(i, 2), path(i+2, 1), path(i+2, 2), obs, r)
            path(i+1, :) = [];
        end
        i = i + 1;
    end
end