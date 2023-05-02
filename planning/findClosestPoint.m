function [closest_pt] = findClosestPoint(list, pt)
% FINDCLOSESTPOINT: find the closest point in list to pt
%
%   INPUTS:
%       list:   k-by-2
%       pt:     1-by-2
%
%   OUTPUTS:  
%       closest_pt: 1-by-2
%
    dists = pdist2(list, pt);
    [~, idx] = min(dists);
    closest_pt = list(idx, :);
end