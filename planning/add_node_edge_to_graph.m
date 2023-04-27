function [nodes, graph] = add_node_edge_to_graph(nodes, graph, node, obs)
% ADD_NODE_EDGE_ TO_GRAPH: Given a graph and its corresponding nodes, add 
% a new node to the graph and its corresponding visible edges.
%
%   INPUTS:
%       nodes   k-by-2 vertices [x, y]
%       graph   a graph, nodes are vertices, edges are lines in Qfree,
%               weights are the lengths of the edges
%       node    1-by-2 new node to add [x, y]
%       obs     1-by-n cell array of n obstacles, each obstacle is k-by-2
%   OUTPUTS:
%       nodes   (k+1)-by-2
%       graph   the graph with the new node and edges added
%
    graph = addnode(graph, 1);
    for i=1:size(nodes, 1)
        if check_free(nodes(i, 1), nodes(i, 2), node(1), node(2), obs)
            graph = addedge(graph, i, size(nodes, 1) + 1, norm(nodes(i, :) - node));
        end
    end
    nodes(end+1, :) = node;
end