function [waypoints_startTOgoal, edges, found_path] = buildRRT(obs,mapBoundary,start,goal, radius)
% BUILDRRT: Run RRT with circular robot
%
%       INPUTS:
%           obs             obs     cell array of obstacles, each ob m-by-2 matrix
%           mapBoundary     1 x 4 vector capturing the [x_bl y_bl x_tr y_tr] coordinates of the bottom left 
%                           and top right corner of the workspace respectively  
%           start           1 x 2 array [x y], for start point
%           goal            1 x 2 array [x y], for goal point
%           radius          scalar, radius of the circular robot
%       OUTPUTS:
%           waypoints       n x 2 array, for a series of waypoints (n waypoints) defining a 
%                           collision-free path from start to goal

%% Parameter Setup
    step_size = 0.2; % Ideal
%     step_size = 0.09;
    max_tree_size = 100;
%     h = figure();
    
%     [pmap] = plotObs(obs, mapBoundary);
    hold on
    nodes = start;
    edges = [0 0 0 0];
    g = graph();
    g = addnode(g, 1);
    found_path = false;
    
    %% Loop
    while size(nodes,1) < max_tree_size
        q_samp = sample_from_Qfree(obs, mapBoundary, goal);
        [q_near, q_near_idx] = find_nearest(nodes, q_samp);
        q_new = extend(q_samp, q_near, step_size, obs);
        if check_circle_free(q_new, obs, radius) && check_rectangle_free(q_near(1), q_near(2), q_new(1), q_new(2), obs, radius)
            nodes(end+1, :) = q_new;
            g = addnode(g, 1);
            g = addedge(g, q_near_idx, size(nodes,1), norm(q_near - q_new));
            edges(end+1, :) = [q_near(1), q_near(2), q_new(1), q_new(2)];
            if isequal(goal, q_new) || check_rectangle_free(q_new(1), q_new(2), goal(1), goal(2), obs, radius)
                if ~isequal(goal, q_new)
                    nodes(end+1, :) = goal;
                    g = addnode(g, 1);
                    g = addedge(g, size(nodes,1)-1, size(nodes,1), norm(q_new - goal));
                    edges(end+1, :) = [q_new(1), q_new(2), goal(1), goal(2)];
                end
                waypoints_startTOgoal = findShortestPath(nodes, g, 1, size(nodes, 1));
                found_path = true;
                break;
            end
        end
    end
    waypoints_startTOgoal = smoothen_path_circular(waypoints_startTOgoal, obs, radius);
%     error('RRT Fails')
end

