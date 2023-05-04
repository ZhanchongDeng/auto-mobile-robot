function [dataStore] = testMovingLocalization_PF(Robot, maxTime)
    % TESTMOVINGLOCALIZATION - controls the robot motion
    %
    %   [dataStore] = testMovingLocalization(Robot,maxTime)
    %
    %   INPUTS
    %       Robot       robot object
    %       maxTime     maximum time to run the control loop (in seconds)
    %
    %   OUTPUTS
    %       dataStore   struct containing all data from the run
    %

    % Set unspecified inputs
    if nargin < 1
        disp('ERROR: TCP/IP port object not provided.');
        return;
    elseif nargin < 2
        maxTime = 50;
    end

    try
        % When running with the real robot, we need to define the appropriate
        % ports. This will fail when NOT connected to a physical robot
        CreatePort = Robot.CreatePort;
    catch
        % If not real robot, then we are using the simulator object
        CreatePort = Robot;
    end

    % declare dataStore as a global variable so it can be accessed from the
    % workspace even if the program is stopped
    global dataStore;

    % initialize datalog struct (customize according to needs)
    dataStore = struct('truthPose', [], ...
        'odometry', [], ...
        'rsdepth', [], ...
        'bump', [], ...
        'beacon', [],...
        'predictedPose',[],...
        'lastBeaconTime', NaN);

    % Variable used to keep track of whether the overhead localization "lost"
    % the robot (number of consecutive times the robot was not identified).
    % If the robot doesn't know where it is we want to stop it for
    % safety reasons.
    noRobotCount = 0;

 % parameter to try
    selfRotateTime = 16;
    maxV = 0.2; % speed of car
    maxW = 0.13; % angular

    pSize = 120; % particle size
    particleStateNoise = [0.1; 0.1; pi / 36]; % noise for spreading particles
    particleDepthNoise = 0.4; % noise for evaluating depth rays
    particleBeaconNoise = 0.05; % noise for evaluating beacon
    k = 5; % top K particles to estimate final pose
    
    h = figure;
    
    % Constants
    sensor_pos = [0 0.08];
    % beaconLoc, map, optWalls, beaconLoc, waypoints, ECwaypoints
    load("practiceMap_4credits/practiceMap_4credits_2023.mat")
    
    % Beacon setup
    beacon_length = size(beaconLoc, 1);
    % swap order
    beacon = [beaconLoc(:, 2) beaconLoc(:, 3) beaconLoc(:, 1)];
    
    % Initialize particles
    initialParticles = zeros(pSize, 3) + [-2.44500000000000 -0.578000000000000 -pi/2];
    dataStore.particles = initialParticles;
    dataStore.weights = 1 / pSize + zeros(pSize, 1);
    
    % anonymous functions
    dynamics = @(x, u) integrateOdom(x, u(1), u(2));
    n_rs_rays = 9;
    angles_degree = linspace(27, -27, n_rs_rays);
    angles = angles_degree * pi / 180;
    sensorDepth = @(x) depthPredict(x, map, sensor_pos, angles.');
    h_depthAndBeacon = @(x) [hBeacon(x, sensor_pos, beacon); depthPredict(x, map, sensor_pos, angles.')];

  
    % Control Loop
    SetFwdVelAngVelCreate(Robot, 0, 0);
    tic


    while toc < maxTime
        % READ & STORE SENSOR DATA
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

        if size(dataStore.odometry, 1) == 1
            dataStore.odometry(1, 2:3) = [0 0];
        end
        
        tp = dataStore.truthPose(end, 2:4)
        % Control Sequence
        bumpInfoNow = dataStore.bump(end, :).';
        bumps = bumpInfoNow([2 3 7]);

        if any(bumps)
            TravelDistCreate(Robot, maxV, -0.5);
            %         travelDist(CreatePort, V, -1);
            TurnCreate(Robot, maxW, -30);
            %         turnAngle(CreatePort, W, -30);
        else
            cmdV = 5;
            cmdW = 0;
        end

        real_time = true;

        % only evaluate these if in real time
        if real_time
            % Collect some data from dataStore
%             tp = dataStore.truthPose(end, 2:end).';

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
            delete(h);
        
            h = plotPF(dataStore, map);
        end

        % Limit the commands
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, maxW);

        % if overhead localization loses the robot for too long, stop it
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(Robot, 0, 0);
        else
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        end

%         delete(h);
%         h = plotEKF(map, dataStore);
%         pause(0.5)

    end

    % set forward and angular velocity to zero (stop robot) before exiting the function
    SetFwdVelAngVelCreate(Robot, 0, 0);
    % Control Loop Ends

    % Data Processing
%     dataStore.deadReck = integrateOdom(dataStore.truthPose(1, 2:end).', dataStore.odometry(:, 2), dataStore.odometry(:, 3));

end
