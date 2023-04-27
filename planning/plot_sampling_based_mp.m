clc; clear;
close all;
%% Plot Cell decomposition
lim = [0 0 100 100];
map_file = 'hw6b.txt';
hdecomp = figure();
[obs,hmap] = plotMap(map_file, lim, hdecomp);
% title('Cell Decomposition')
hold on

%% Roadmap
[nodes, edges, g] = createRoadmap(obs, lim);
prm = plotmap(edges, false);
xlabel('x (m)')
ylabel('y (m)')
legend([hmap(1), prm(1)], ...
    'Map', 'Roadmap');
title('Roadmap')
hold on

%% Shortest path in roadmap
% start = [22 65];
% goal = [90 10];
start = [80 95];
goal = [10 50];
path = findPath(obs, nodes, g, start, goal);
ppath = plot(path(:, 1), path(:, 2), 'r-', 'LineWidth', 2);
pstart = plot(start(1), start(2), 'co', 'MarkerSize', 10);
pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
legend([hmap(1), prm(1), ppath, pstart, pend], ...
    'Map', 'Roadmap', 'Path', 'Start', 'Goal');
title("Shortest Path from [" + num2str(start) + "] to [" + num2str(goal) +"]")
hold off

%% Test sample_from_Qfree
sample_from_Qfree(obs, lim, [20 65])

%% Test extend
q_samp = [50 40];
q_near = [60 60];
step_size = 0.5;
q_new = extend(q_samp, q_near, step_size, obs);
% plot([60;50;40], [60;40;20])

%% Test buildRRTpoint
start = [20 65];
goal = [90 10];
map = 'hw6b.txt';
mapBoundary = [0 0 100 100];
[waypoints_startTOgoal] = buildRRTpoint(map,mapBoundary,start,goal);

%% Test check_circle_free
pt = [80 60];
r = 0.5;
is_free = check_circle_free(pt, obs, r);

%% Test build_rectangle
x1 = 0;
y1 = 0;
x2 = 30;
y2 = 20;
r = 1;
rec = build_rectangle(x1, y1, x2, y2, r)

%% Test check_rectangle_free
x1 = 0;
y1 = 0;
x2 = 10;
y2 = 10;
r = 1;
[is_free] = check_rectangle_free(x1, y1, x2, y2, obs, r)

%% Test buildRRT (circular)
start = [20 65];
goal = [90 10];
radius = 1;
map = 'hw6b.txt';
mapBoundary = [0 0 100 100];
[waypoints_startTOgoal] = buildRRT(obs,mapBoundary,start,goal, radius);

%% Test wall2obs
wall2obs(m, 0.2)

%% TestRRT
start = [20 65];
goal = [90 10];
radius = 1;
map = 'hw6b.txt';
mapBoundary = [0 0 100 100];
TestRRT(map,mapBoundary,start,goal,radius)

%% ==== Lab 4 ==== %%
%% Setup
map_file = 'labBoxMap_wall_orange.mat';
goal = load(map_file).goal1;
map = load(map_file).map;
mapBoundary = map(1, :);
wallmap = map(2:end, :);
radius = 0.1;

%% Test buildRRT circular robot
obs = wall2obs(wallmap, radius);
start = [-1 0];
[waypoints, edges, pmap] = buildRRT(obs,mapBoundary,start,goal, radius);

%% Plot RRT and waypoints
prrt = plotmap(edges, false);
pwaypoints = plot(waypoints(:, 1), waypoints(:, 2), 'r-', 'LineWidth', 1);
pstart = plot(start(1), start(2), 'co', 'MarkerSize', 10);
pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
 
    
legend([pmap(1), prrt(1), pwaypoints, pstart, pend], ...
        'Map', 'Search Tree', 'Path', 'Start', 'Goal');
xlabel('x(m)')
ylabel('y(m)')
title("RRT tree and path from [" + string(start(1)) + "," + string(start(2)) + "] to [" + string(goal(1)) + "," + string(goal(2)) +"], radius = " + string(radius))
hold off

%% ==== Lab 4 Report ==== %
%% Setup
map_file = 'labBoxMap_wall_orange.mat';
goal = load(map_file).goal1;
map = load(map_file).map;
mapBoundary = map(1, :);
wallmap = map(2:end, :);
radius = 0.16;
truthPose = dataStore.truthPose;

%% Plot map
h = figure();
[pmap] = plotmap(map, false);
hold on
%% Plot trajectory
for i=1:size(truthPose, 1)
    ptraj = viscircles([truthPose(i, 2), truthPose(i, 3)], radius);
    hold on
end
legend([pmap(1), ptraj], ...
        'Map', 'Trajectory')
xlabel('x(m)')
ylabel('y(m)')
title("RRT trajectory radius = " + string(radius))
hold off
