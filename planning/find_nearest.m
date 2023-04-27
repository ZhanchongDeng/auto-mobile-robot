function [q_near, q_near_idx] = find_nearest(nodes, q_samp)
% FIND_NEAREST: Find the nearest node to q_samp
%
%   INPUTS:
%       nodes       k-by-2 vertices [x, y]
%       q_samp      1-by-2 target pt [x y]
%
%   OUTPUTS:
%       q_near      1-by-2 nearest node to q_samp
%       q_near_idx  index of the nearest node in nodes
    distances = pdist2(nodes, q_samp);
    [~, q_near_idx] = min(distances);
    q_near = nodes(q_near_idx, :);
end