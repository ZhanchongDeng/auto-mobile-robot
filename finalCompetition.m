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
                   'particles', [], ...
                   'particle_weights', [], ...
                   'particles_bar', [], ...
                   'GPS', [], ...
                   'visitedWaypoints', [], ...
                   'lastBeaconTime', NaN);
    
    % Parameter Setup
    flag_use_truthpose = false;
    noRobotCount = 0;
    wheel2Center = 0.16;
    radius = 0.2;
    maxV = 0.15;
    epsilon = 0.13;
    closeEnough = 0.25;
    sensor_pos = [offset_x, offset_y];
%     sensor_pos = [0,0.08];
    
    % Load map
    map_file = 'compMap.mat';
    load(map_file, 'beaconLoc', 'ECwaypoints', 'map', 'optWalls', 'waypoints');
    % map(end+1, :) = optWalls(2, :);
    
    %% ==== Plot map ==== %%
    pmap = plotmapopts(map, 'color', 'black', 'LineWidth', 2);
    hold on
    poptwalls = plotmapopts(optWalls, 'color', 'red', 'LineWidth', 2);
    hold on
    
    %% ==== Initial Localization Setup ==== %%
    selfRotateTime = 16;
    initialLocalizationTime = 18;
    pSize = 120;
    particleStateNoise = [0.01; 0.01; pi / 50]; % noise for spreading particles
    particleDepthNoise = 1; % noise for evaluating depth rays
    particleBeaconNoise = 0.1; % noise for evaluating beacon
    k = 5; % top K particles to estimate final pose   
    
    % Initialize particles
    initialParticles = particlesFromWaypoints(pSize, waypoints);
    dataStore.particles = initialParticles;
    dataStore.weights = 1/pSize + zeros(pSize,1);
    
    % Beacon setup
    beacon = [beaconLoc(:, 2) beaconLoc(:, 3) beaconLoc(:, 1)];
    
    % anonymous functions
    dynamics = @(x,u) integrateOdom(x, u(1), u(2));
    n_rs_rays = 9;
    angles_degree = linspace(27, -27, n_rs_rays);
    angles = angles_degree * pi / 180;
    h_depthAndBeacon = @(x) [hBeacon(x, sensor_pos, beacon); depthPredict(x, map, optWalls, sensor_pos, angles.')];
    
    %% ==== Initial Localization Control Loop ==== %% 
    tic
    while toc < initialLocalizationTime
%         
%         % READ & STORE SENSOR DATA
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

        if size(dataStore.odometry, 1) == 1
            dataStore.odometry(:, 2:end) = 0;
        end


        % get control and detph
        u = dataStore.odometry(end, 2:end).';
        z_depth = dataStore.rsdepth(end, 3:end).';
        [z_beacon, dataStore.lastBeaconTime] = getBeacon(dataStore, beacon);
         doSample = size(dataStore.odometry,1) ~= 1;

        currentParticles = dataStore.particles(:, :, end);
        currentWeights = dataStore.weights(:, :, end);
        [dataStore.particles(:, :, end + 1), dataStore.weights(:, :, end + 1)] = ...
            PF_beacon(currentParticles, currentWeights, ...
            particleStateNoise, particleBeaconNoise, particleDepthNoise, ...
            u, z_depth, z_beacon, dynamics, h_depthAndBeacon, doSample);

        if toc < selfRotateTime
            turnAngle(CreatePort, 0.2, 45);
        end
       
        SetFwdVelAngVelCreate(Robot, 0, 0);
    end

    % Extract pose
    dataStore.predictedPose = matchWaypoints(dataStore.particles(:, :, end), dataStore.weights(:, :, end), waypoints, k);
    dataStore.predictedPose(1,3) = mod(dataStore.predictedPose(1,3),pi);
    
    %% ==== PF Moving Localization Setup ==== %% 
    particleStateNoise = [0.1; 0.1; pi / 36]; % noise for spreading particles
    particleDepthNoise = 0.4; % noise for evaluating depth rays
    particleBeaconNoise = 0.05; % noise for evaluating beacon
    
    % Initialize particles
    initialParticles = zeros(pSize, 3) + dataStore.predictedPose(1,3);
    dataStore.particles(:, :, end + 1) = initialParticles;
    dataStore.weights(:, :, end + 1) = 1 / pSize + zeros(pSize, 1);
    
    %% ==== Planning setup ==== %%
    % assume optionalWalls are real for planning ONLY
    mapWithOptional = [map; optionalWalls];

    dataStore.unvisitedWaypoints = [waypoints; ECwaypoints];
    dataStore.visitedWaypoints = [];
    mapBoundary = [min(min(map(:,1)), min(map(:,3))), ...
                   min(min(map(:,2)), min(map(:,4))), ...
                   max(max(map(:,1)), max(map(:,3))), ...
                   max(max(map(:,2)), max(map(:,4)))];
    obs = wall2obs(mapWithOptional, radius);
%     [pmap] = plotObs(obs, mapBoundary);
%     hold on

    %% ==== Check out initial waypoint ==== %%
    if flag_use_truthpose
        [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
        curr_pos = dataStore.truthPose(end, 2:3);
    else
        curr_pos = dataStore.predictedPose(1:2);
    end
    closest_point = findClosestPoint(dataStore.unvisitedWaypoints, curr_pos);
    dataStore.visitedWaypoints(end+1, :) = closest_point;
    dataStore.unvisitedWaypoints = removePointFromList(dataStore.unvisitedWaypoints, closest_point);
    pvisited = plot(closest_point(1), closest_point(2), 'r*', 'MarkerSize', 10);
    BeepCreate(Robot)
    
%     tStart = tic
%     disp('start running TSP')
    dataStore.unvisitedWaypoints = sort_list_by_TSP(dataStore.unvisitedWaypoints, closest_point);
%     disp('finish running TSP')

    %% ==== RRT to visit unvisited waypoints ==== %% 
    while toc < maxTime && size(dataStore.unvisitedWaypoints, 1) > 0
        if flag_use_truthpose
            start = dataStore.truthPose(end, 2:3);
        else
            start = dataStore.predictedPose(end, 1:2);
        end
%         goal = findClosestPoint(dataStore.unvisitedWaypoints, start);
        goal = dataStore.unvisitedWaypoints(1, :);
        [waypoints, edges] = buildRRT(obs,mapBoundary,start,goal, radius);
        disp('waypoints')
        disp(waypoints)
        prrt = plotmap(edges, false);
        pwaypoints = plot(waypoints(:, 1), waypoints(:, 2), 'r-', 'LineWidth', 1);
        pstart = plot(start(1), start(2), 'co', 'MarkerSize', 10);
        pend = plot(goal(1), goal(2), 'm*', 'MarkerSize', 10);
        gotopt = 1;
        
        %% Control loop
        while toc < maxTime

            % READ & STORE SENSOR DATA
            [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

            % GET ROBOT POSE
            if flag_use_truthpose
                pose = dataStore.truthPose(end, 2:end);
            else
                % get control and detph
                u = dataStore.odometry(end, 2:end).';
                z_depth = dataStore.rsdepth(end, 3:end).';
                [z_beacon, dataStore.lastBeaconTime] = getBeacon(dataStore, beacon);

                currentParticles = dataStore.particles(:, :, end);
                currentWeights = dataStore.weights(:, :, end);

                [dataStore.particles(:, :, end + 1), dataStore.weights(:, :, end + 1)] = ...
                    PF_beacon(currentParticles, currentWeights, ...
                    particleStateNoise, particleBeaconNoise, particleDepthNoise, ...
                    u, z_depth, z_beacon, dynamics, h_depthAndBeacon, true);

                dataStore.predictedPose  = [dataStore.predictedPose ; topKPose(dataStore.particles(:, :, end), dataStore.weights(:, :, end), k)];
                pose = dataStore.predictedPose(end, 1:3);
            end
            
            % CONTROL FUNCTION (send robot commands)
            [cmdV, cmdW, gotopt] = visitWaypoints(waypoints, pose, gotopt, closeEnough, epsilon);

            if gotopt > length(waypoints)
%                 SetFwdVelAngVelCreate(Robot, 0, 0);
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
        dataStore.visitedWaypoints(end+1, :) = goal;
        dataStore.unvisitedWaypoints = removePointFromList(dataStore.unvisitedWaypoints, goal);
        pvisited = plot(goal(1), goal(2), 'r*', 'MarkerSize', 10);
        BeepCreate(Robot)
        
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