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
    noRobotCount = 0;
    wheel2Center = 0.16;
    radius = 0.15;
    maxV = 0.1;
    epsilon = 0.2;
    gotopt = 1;
    closeEnough = 0.2;
    sensor_pos = [offset_x, offset_y];
    
    % Load map
    map_file = 'practiceMap_4credits.mat';
    load(map_file, 'beaconLoc', 'ECwaypoints', 'map', 'optWalls', 'waypoints');
    
    %% ==== Plot map ==== %%
    pmap = plotmapopts(map, 'color', 'green', 'LineWidth', 2);
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
    

end