function TestRRT(map,mapBoundary,start,goal,radius)
% Test function for MAE 4180/5180, CS 4758/5758, ECE 4180/5772 HW 6 RRT
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
%	        radius          robot radius in m

%       OUTPUTS:
%           plot of the workspace with an RRT created for connecting the start and the goal    
    [obs, ~] = plotMap(map, mapBoundary);
    hold on
    [waypoints_startTOgoal, edges, pmap] = buildRRT(obs,mapBoundary,start,goal, radius);
    %% Plot RRT and waypoints
    prrt = plotmap(edges, false);
    hold on
    pwaypoints = plot(waypoints_startTOgoal(:, 1), waypoints_startTOgoal(:, 2), 'r-', 'LineWidth', 1);
    hold on
    pstart = plot(start(1), start(2), 'co', 'MarkerSize', 10);
    hold on
    pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
    %% Plot
   
    legend([pmap(1), prrt(1), pwaypoints, pstart, pend], ...
            'Map', 'Search Tree', 'Path', 'Start', 'Goal');
    xlabel('x(m)')
    ylabel('y(m)')
    title("RRT tree and path from [" + string(start(1)) + "," + string(start(2)) + "] to [" + string(goal(1)) + "," + string(goal(2)) +"], radius = " + string(radius))
    hold off
end