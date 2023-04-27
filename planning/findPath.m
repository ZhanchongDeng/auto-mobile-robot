function [path] = findPath(obs, nodes, graph, start, goal)
% FINDPATH: Given a polygonal environment, a roadmap, and initial and goal 
% points, returns the shortest path connecting the initial and goal points
%
%   INPUTS:
%       obs     1-by-n cell array of n obstacles, each obstacle is k-by-2
%       nodes   k-by-2 vertices [x, y]
%       graph   a graph, nodes are vertices, edges are lines in Qfree,
%               weights are the lengths of the edges
%
%   OUTPUTS:
%       path    m-by-2 list of vertices [x y] from start to goal
%       
    %% Add start and goal to the graph
%     nodes(end+1:end+2, :) = vertcat(start, goal);
    [nodes, graph] = add_node_edge_to_graph(nodes, graph, start, obs);
    [nodes, graph] = add_node_edge_to_graph(nodes, graph, goal, obs);
    
    %% Compute the shortest path
    path = findShortestPath(nodes, graph, size(nodes, 1) - 1, size(nodes, 1));
%     P = shortestpath(graph, size(nodes, 1) - 1, size(nodes, 1));
%     d = length(P);
%     path = zeros(d, 2);
%     for i=1:d
%         path(i, :) = nodes(P(i), :);
%     end
end