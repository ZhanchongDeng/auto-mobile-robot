function [nodes, edges, g] = createRoadmap(obs, bound)
% CREATEROADMAP: Given vertices of obstacles, retrn a roadmap covering
% Qfree. Use visibility graph method
%
%   INPUTS:
%       obs     1-by-n cell array of n obstacles, each obstacle is k-by-2
%       bound   1-by-4 rectangle boundary of workspace [x1 y1 x2 y2]
%
%   OUTPUTS:
%       nodes   k-by-2 vertices [x, y]
%       edges   m-by-4 edges [x1 y1 x2 y2]
%       g       a graph, nodes are vertices, edges are lines in Qfree,
%               weights are the lengths of the edges
    %% Collect all vertices 
    x1 = bound(1);
    y1 = bound(2);
    x2 = bound(3);
    y2 = bound(4);
    vertices = vertcat(obs{:});
    nodes = vertcat(vertices, [x1 y1; x1 y2; x2 y1; x2 y2]);
    g = graph();
    g = addnode(g, size(nodes, 1));
    
    %% Compute all edges
    edges = [0 0 0 0];
    for i=1:size(nodes, 1)
        for j=i+1:size(nodes, 1)
            if check_free(nodes(i, 1), nodes(i, 2), nodes(j, 1), nodes(j, 2), obs)
                edges(end+1, :) = [nodes(i, 1), nodes(i, 2), nodes(j, 1), nodes(j, 2)];
                g = addedge(g, i, j, norm(nodes(i, :) - nodes(j, :)));
            end
        end
    end
    edges = edges(2:end, :);
end