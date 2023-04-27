function [path] = findShortestPath(nodes, graph, start, goal)
% FINDSHORTESTPATH: Given nodes, a graph, start and goal pts, find the
% shortest path 
%   INPUTS:
%       nodes   k-by-2 vertices [x, y]
%       graph   a graph, nodes are vertices, edges are lines in Qfree,
%               weights are the lengths of the edges
%       start   int index of the start node
%       goal    int index of the goal node
%
%   OUTPUTS:
%       path    m-by-2 list of vertices [x y] from start to goal
    P = shortestpath(graph, start, goal);
    d = length(P);
    path = zeros(d, 2);
    for i=1:d
        path(i, :) = nodes(P(i), :);
    end
end