function [dataStore] = initialLocalization(Robot,maxTime)
% initialLocalization: Determine which waypoint the robot is on using Particle Filter.
%
%   [dataStore] = initialLocalization(Robot,maxTime)
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
    maxTime = 2;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

% parameter to try
maxV = 0.4;             % speed of car
maxW = 0.13;            % angular
pSize = 100;            % particle size
particleStateNoise = 0.01;  % noise for spreading particles
particleSensorNoise = 0.01; % noise for evaluating particles
k = 5;                 % top K particles to estimate final pose
% Constants
sensor_pos = [0 0];
% beaconLoc, map, optWalls, beaconLoc, waypoints, ECwaypoints
load("practiceMap_4credits/practiceMap_4credits.mat")

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

% Control Loop
SetFwdVelAngVelCreate(Robot, 0,0);
tic
while toc < maxTime
    
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

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0, 0);
% Control Loop Ends

% Extract pose
dataStore.predictedPose = matchWaypoints(dataStore.particles(:,:,end), dataStore.weights(:,:,end), waypoints, k);
end

