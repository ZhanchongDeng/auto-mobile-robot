function [dataStore] = rrtPlanner(Robot, maxTime)
 %% Setup %%
    if nargin < 1
        disp('Error: TCP/IP port object not provided.')
    elseif nargin < 2
        maxTime = 800;
    end
    
    try
        CreatePort = Robot.CreatePort;
    catch
        CreatePort = Robot;
    end
    
    global dataStore;
    
    dataStore = struct('truthPose', [], ...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'deadReck', [], ...
                   'ekfMu', [], ...
                   'ekfSigma', [], ...
                   'particles', [], ...
                   'particle_weights', [], ...
                   'particles_bar', [], ...
                   'GPS', []);
    SetFwdVelAngVelCreate(Robot, 0, 0)
    
    % Parameter Setup
    noRobotCount = 0;
    wheel2Center = 0.16;
    radius = 0.15;
    maxV = 0.1;
    epsilon = 0.2;
%     map_file = 'cornerMap';
    map_file = 'labBoxMap_wall_orange.mat';
    goal = load(map_file).goal1;
    gotopt = 1;
    closeEnough = 0.2;
    
    map = load(map_file).map;
    mapBoundary = [min(min(map(:,1)), min(map(:,3))), ...
                   min(min(map(:,2)), min(map(:,4))), ...
                   max(max(map(:,1)), max(map(:,3))), ...
                   max(max(map(:,2)), max(map(:,4)))];
%     mapBoundary = map(1, :);
    wallmap = map;
    obs = wall2obs(wallmap, radius);
    
    
    %% RRT
    tic
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    start = dataStore.truthPose(end, 2:3);
    [waypoints, edges, pmap] = buildRRT(obs,mapBoundary,start,goal, radius);
    
    %% Plot RRT and waypoints
    prrt = plotmap(edges, false);
    pwaypoints = plot(waypoints(:, 1), waypoints(:, 2), 'r-', 'LineWidth', 1);
    pstart = plot(start(1), start(2), 'co', 'MarkerSize', 10);
    pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
    
    
    %% Control loop
    while toc < maxTime
    
        % READ & STORE SENSOR DATA
        [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
        % CONTROL FUNCTION (send robot commands)
        pose = dataStore.truthPose(end, 2:end);
        [cmdV, cmdW, gotopt] = visitWaypoints(waypoints, pose, gotopt, closeEnough, epsilon);
    
        if gotopt > length(waypoints)
            SetFwdVelAngVelCreate(Robot, 0, 0);
            disp("Reach the end of waypoints")
            break
        end
    
        % if overhead localization loses the robot for too long, stop it
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(Robot, 0,0);
        else
            [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        end
    
    end

    % set forward and angular velocity to zero (stop robot) before exiting the function
    SetFwdVelAngVelCreate(Robot, 0, 0);
    %% Plot
    poses = dataStore.truthPose(:, 2:3);
    ptraj = plot(poses(:, 1), poses(:, 2), 'k-', 'LineWidth', 2);
    legend([pmap(1), prrt(1), pwaypoints, ptraj, pstart, pend], ...
            'Map', 'Search Tree', 'Path', 'Trajectory', 'Start', 'Goal');
    xlabel('x(m)')
    ylabel('y(m)')
    title("RRT tree, path, and trajectory from [" + string(start(1)) + "," + string(start(2)) + "] to [" + string(goal(1)) + "," + string(goal(2)) +"], radius = " + string(radius))
    hold off
end
    
    
    
    