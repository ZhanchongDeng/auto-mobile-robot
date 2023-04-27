function [waypoints_startTOgoal] = buildRRTpoint(map,mapBoundary,start,goal)
% BUILDRRTPOINT
% AMR Homework 6 
%
%       INPUTS:
%           map             file name for text file representing the obstacles in the workspace
%                           for example map = 'hw6b.txt'. Each row in this file contains the vertices
%                           of one polygonal obstacle: v1x, v1y, v2x, v2y, etc. The vertices are given in
%                           counterclockwise order. If an obstacle has fewer vertices, unused entries 
%                           in the line will contain the value zero.
%           mapBoundary     1 x 4 vector capturing the [x_bl y_bl x_tr y_tr] coordinates of the bottom left 
%                           and top right corner of the workspace respectively  
%           start           1 x 2 array [x y], for start point
%           goal            1 x 2 array [x y], for goal point
%       OUTPUTS:
%           waypoints       n x 2 array, for a series of waypoints (n waypoints) defining a 
%                           collision-free path from start to goal                           
% Autonomous Mobile Robots
    %% Parameter Setup
    step_size = 1;
    max_tree_size = 5000;
    h = figure();
    
    [obs, pmap] = plotMap(map, mapBoundary);
    hold on
    nodes = start;
    edges = [0 0 0 0];
    g = graph();
    g = addnode(g, 1);
    
    %% Loop
    while size(nodes,1) < max_tree_size
        q_samp = sample_from_Qfree(obs, mapBoundary, goal);
        [q_near, q_near_idx] = find_nearest(nodes, q_samp);
        q_new = extend(q_samp, q_near, step_size, obs);
        if check_pt_free(q_new, obs) && check_free(q_near(1), q_near(2), q_new(1), q_new(2), obs)
            nodes(end+1, :) = q_new;
            g = addnode(g, 1);
            g = addedge(g, q_near_idx, size(nodes,1), norm(q_near - q_new));
            edges(end+1, :) = [q_near(1), q_near(2), q_new(1), q_new(2)];
            if isequal(goal, q_new) || check_free(q_new(1), q_new(2), goal(1), goal(2), obs)
                if ~isequal(goal, q_new)
                    nodes(end+1, :) = goal;
                    g = addnode(g, 1);
                    g = addedge(g, size(nodes,1)-1, size(nodes,1), norm(q_new - goal));
                    edges(end+1, :) = [q_new(1), q_new(2), goal(1), goal(2)];
                end
                waypoints_startTOgoal = findShortestPath(nodes, g, 1, size(nodes, 1));
                break;
            end
        end
    end
    %% Plot RRT and waypoints
    waypoints_startTOgoal = smoothen_path(waypoints_startTOgoal, obs);
    prrt = plotmap(edges, false);
    pwaypoints = plot(waypoints_startTOgoal(:, 1), waypoints_startTOgoal(:, 2), 'r-', 'LineWidth', 1);
    pstart = plot(start(1), start(2), 'co', 'MarkerSize', 10);
    pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
    legend([pmap(1), prrt(1), pwaypoints, pstart, pend], ...
            'Map', 'Search Tree', 'Path', 'Start', 'Goal');
    title("RRT Search Tree and Full Path from [" + num2str(start) + "] to [" + num2str(goal) +"]")
    hold off
%     error('RRT Fails')
end