function [optimal_path] = sort_list_by_TSP(list, curr)
% SORT_LIST_BY_TSP: sort the list of 2d points by brute-force travelling
% salesman algorithm
%
%   INPUTS:
%       list:           k-by-2 pts
%       curr:           1-by-2 pt
%
%   OUTPUTS:
%       optimal_path:    k-by-2 pts
%
    n = size(list, 1);
    permutation = perms(1:n);
    min_dist = Inf;
    optimal_path = [];
    for i=1:size(permutation, 1)
        path_idx = permutation(i, :);
        path = list(path_idx, :);
        dist = 0;
        for j=1:size(path, 1)-1
            dist = dist + norm(path(j,:) - path(j+1, :));
        end
        dist = dist + norm(curr - path(1, :));
        if dist < min_dist
            min_dist = dist;
            optimal_path = path;
        end
    end
end