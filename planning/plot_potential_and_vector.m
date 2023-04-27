clc;
close all;
%% Parameter setup
c_att = 0.1;
c_rep = 5;
Q = 5;
n = 50;
m = 50;
map = importdata('hw6Sphere.txt');
goal = [30 40];


%% Plot
[pointsX, pointsY] = meshgridCircle2(bound, n, m);
[h_potential_field, h_vector_field, U, U_grad] = potentialPlot(map, goal, c_att, c_rep, Q, pointsX, pointsY);
% disp(U)

%% Parameter setup for trajectory
t_delta = 0.5;
points = [80, 55];
grads = [0 0];
finish_threshold = 0.1;

%% Compute the trajectory
while ~close_enough(points(end, :), goal, finish_threshold)
    [U, U_grad] = potentialPoint(map, goal, c_att, c_rep, Q, points(end, :));
    points(end+1, :) = points(end, :) - U_grad * t_delta;
    grads(end+1, :) = U_grad;
end

%% Plot the trajectory
pboundary = circle(map(1, 1:2), map(1, 3), 1000);
hold on
for i=2:size(map, 1)
    pobstacle = circle(map(i, 1:2), map(i, 3), 1000, 'g-');
    hold on
end
% pvector = quiver(points(1:end-1,1), points(1:end-1,2), grads(2:end, 1), grads(2:end, 2));
pvector = plot(points(:, 1), points(:, 2), 'r-');
hold on
pstart = plot(points(1,1),points(1,2), 'co', 'MarkerSize', 10);
pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
hold on
xlabel('x (m)')
ylabel('y (m)')
legend([pboundary, pobstacle, pvector, pstart, pend], ...
        'Boundary', 'Obstacles', 'Trajectory', 'Start', 'Goal');
title('Robot Trajectory start at [' + string(points(1, 1)) + ',' + string(points(1, 2)) + '], end at [' + string(goal(1)) + ',' + string(goal(2))+'], time step = ' + string(t_delta))
hold off

%% TestSphereWorldPlot
points = meshgridCircle(bound)
TestSphereWorldPot(map, goal, c_att, c_rep, Q, points)

%% Simulate Create

% Create sphere world
sphere_world = [0 0 6; ...
                -2.974 -2.045 1.44; ...
                0.026 2.99 1.41; ...
                2.5 -2.52 0.74];
% plotSphereWorld(sphere_world, false);

%% Plot Trajectory [-4, 4] / [-4, -4] to [0, 0]
start = [-4, 4];
goal = [0, 0];
hmap = figure();
pboundary = circle(sphere_world(1, 1:2), sphere_world(1, 3), 1000);
hold on
for i=2:size(sphere_world, 1)
    pobstacle = circle(sphere_world(i, 1:2), sphere_world(i, 3), 1000, 'g-');
    hold on
end
points = dataStore.truthPose(:, 2:3);
ptrajectory = plot(points(:, 1), points(:, 2), 'r', 'LineWidth', 2);
pstart = plot(start(1), start(2), 'co', 'MarkerSize', 10);
pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
xlabel('x (m)')
ylabel('y (m)')
legend([pboundary, pobstacle, ptrajectory, pstart, pend], ...
        'Boundary', 'Obstacles', 'Trajectory', 'Start', 'Goal')
title('Trajectory of Create Robot start at [' + string(start(1)) + ',' + string(start(2)) + '], end at [' + string(goal(1)) + ',' + string(goal(2))+']')

%% ==== Lab4: Potential function ==== %%
%% Parameter setup: no local min
clc;close all;clear;
load('labSphereMap_wall_orange.mat')
% mapBoundary = map2boundary(map)
bound = map(1, :);
% map = map(2:end, :);
goal = goal1;

c_att = 0.1;
c_rep = 5;
Q = 0.2;

%% Plot the potential field
points = meshgridCircle(bound);
TestSphereWorldPot(map, goal, c_att, c_rep, Q, points)

%% Parameter setup: local min
clc;close all;clear;
load('labSphereMap_wall_orange.mat')
% mapBoundary = map2boundary(map)
bound = map(1, :);
% map = map(2:end, :);
goal = goal1;

c_att = 1e-5;
c_rep = 2;
Q = 2.3;

%% Plot the potential field
points = meshgridCircle(bound);
TestSphereWorldPot(map, goal, c_att, c_rep, Q, points)

%% Plot Trajectory from Simulator
start = dataStore.truthPose(1, 2:3);
sphere_world = map;
hmap = figure();
pboundary = circle(sphere_world(1, 1:2), sphere_world(1, 3), 1000);
hold on
for i=2:size(sphere_world, 1)
    pobstacle = circle(sphere_world(i, 1:2), sphere_world(i, 3), 1000, 'g-');
    hold on
end
points = dataStore.truthPose(:, 2:3);
ptrajectory = plot(points(:, 1), points(:, 2), 'r', 'LineWidth', 2);
pstart = plot(start(1), start(2), 'co', 'MarkerSize', 10);
pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
xlabel('x (m)')
ylabel('y (m)')
legend([pboundary, pobstacle, ptrajectory, pstart, pend], ...
        'Boundary', 'Obstacles', 'Trajectory', 'Start', 'Goal')
title('Trajectory of Create Robot start at [' + string(start(1)) + ',' + string(start(2)) + '], end at [' + string(goal(1)) + ',' + string(goal(2))+']')

