function [dataStore] = finalCompetition(Robot, maxTime, offset_x, offset_y)
% FINALCOMPETITION: main control function for the final competition
    %% ==== Robot setup ==== %%
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
    SetFwdVelAngVelCreate(Robot, 0, 0)
    
    %% ==== Data setup ==== %%
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
                   'GPS', [], ...
                   'visitedWaypoints', []);
    
    % Parameter Setup
    flag_use_truthpose = true;
    noRobotCount = 0;
    wheel2Center = 0.16;
    radius = 0.15;
    maxV = 0.1;
    epsilon = 0.2;
    gotopt = 1;
    closeEnough = 0.2;
%     sensor_pos = [offset_x, offset_y];
    sensor_pos = [0,0.08];
    
    % Load map
    map_file = 'practiceMap_4credits_2023.mat';
    load(map_file, 'beaconLoc', 'ECwaypoints', 'map', 'optWalls', 'waypoints');
    
    %% ==== Plot map ==== %%
    pmap = plotmapopts(map, 'color', 'black', 'LineWidth', 2);
    hold on
    poptwalls = plotmapopts(optWalls, 'color', 'red', 'LineWidth', 2);
    hold on
    
    %% ==== Particle setup ==== %%
    particle_run_time = 20;
    pSize = 100;
    particleStateNoise = 0.01;  % noise for spreading particles
    particleSensorNoise = 0.01; % noise for evaluating particles
    k = 5;                 % top K particles to estimate final pose
    
    % Initialize particles
    initialParticles = particlesFromWaypoints(pSize, waypoints);
    dataStore.particles = initialParticles;
    dataStore.weights = 1/pSize + zeros(pSize,1);
    
    % anonymous functions
    dynamics = @(x,u) integrateOdom(x, u(1), u(2));
    n_rs_rays = 9;
    angles_degree = linspace(27, -27, n_rs_rays);
    angles = angles_degree * pi / 180;
    sensorDepth = @(x) depthPredict(x, map, sensor_pos, angles.');
    
    %% ==== Particle Control Loop ==== %% 
    tic
    while toc < particle_run_time
        
        % READ & STORE SENSOR DATA
        [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
        if size(dataStore.odometry,1) == 1
            dataStore.odometry(:,2:end) = 0;
        end

        % Spin for some time
        cmdV = 0;
        cmdW = 0.5;

        % get control and detph
        u = dataStore.odometry(end, 2:end).';
        z_depth = dataStore.rsdepth(end,3:end).';

        currentParticles = dataStore.particles(:,:,end);
        currentWeights = dataStore.weights(:,:,end);
        [dataStore.particles(:,:,end+1), dataStore.weights(:,:,end+1)] = ...
            PF(currentParticles, currentWeights, particleStateNoise, particleSensorNoise, u, z_depth, dynamics, sensorDepth);

        % Limit the commands
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, maxW);

        % if overhead localization loses the robot for too long, stop it
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(Robot, 0, 0);
        else
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        end
        pause(0.1);
    end
    SetFwdVelAngVelCreate(Robot, 0, 0);
    dataStore.predictedPose = matchWaypoints(dataStore.particles(:,:,end), dataStore.weights(:,:,end), waypoints, k);
    
    %% ==== EKF Setup ==== %% 
    process_noise = 0.01;
    depth_sensor_noise = 0.01;
    beacon_sensor_noise = 0.01;
    errorThreshold = 0.5;   % slice out all (actual - expected) > threshold
    dataStore.ekfSigma = [0.05 0 0; 0 0.05 0; 0 0 0.1];
    
    %% ==== Planning setup ==== %% 
    dataStore.unvisited_waypoints = [waypoints; ECwaypoints];
    dataStore.visited_waypoints = [];
    mapBoundary = [min(min(map(:,1)), min(map(:,3))), ...
                   min(min(map(:,2)), min(map(:,4))), ...
                   max(max(map(:,1)), max(map(:,3))), ...
                   max(max(map(:,2)), max(map(:,4)))];
    obs = wall2obs(map, radius);
%     [pmap] = plotObs(obs, mapBoundary);
%     hold on
    %% ==== Check out initial waypoint ==== %%
    if flag_use_truthpose
        [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
        curr_pos = dataStore.truthPose(end, 2:3);
    else
        curr_pos = dataStore.predictedPose(1:2);
    end
    closest_point = findClosestPoint(dataStore.unvisited_waypoints, curr_pos);
    dataStore.visited_waypoints(end+1, :) = closest_point;
    dataStore.unvisited_waypoints = removePointFromList(dataStore.unvisited_waypoints, closest_point);
    pvisited = plot(closest_point(1), closest_point(2), 'r*', 'MarkerSize', 10);
    BeepRoomba(Robot)
    
    %% ==== RRT to visit unvisited waypoints ==== %% 
    while toc < maxTime && size(dataStore.unvisited_waypoints, 1) > 0
        start = dataStore.truthPose(end, 2:3);
        goal = findClosestPoint(dataStore.unvisited_waypoints, start);
        [waypoints, edges] = buildRRT(obs,mapBoundary,start,goal, radius);
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
        SetFwdVelAngVelCreate(Robot, 0, 0);
        delete(prrt)
        delete(pwaypoints)
        delete(pstart)
        delete(pend)
        dataStore.visited_waypoints(end+1, :) = goal;
        dataStore.unvisited_waypoints = removePointFromList(dataStore.unvisited_waypoints, goal);
        pvisited = plot(goal(1), goal(2), 'r*', 'MarkerSize', 10);
        BeepRoomba(Robot)
        
    end
    SetFwdVelAngVelCreate(Robot, 0, 0)

    %% ==== Plot Trajectory ==== %
    tp = dataStore.truthPose;
    ptraj = plot(tp(:, 2), tp(:, 3), 'b', 'LineWidth', 2);
%     pekf_mu = plot()
    legend([pmap(1), poptwalls(1), ptraj, pvisited], ...
        'Map', 'Optional Walls', 'Trajectory', 'Visited Waypoints')
    xlabel('x(m)')
    ylabel('y(m)')
    title('Robot true trajectory, EKF estimated trajectory')
    hold off

end