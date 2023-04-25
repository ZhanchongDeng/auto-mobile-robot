% parameter to try
process_noise = 0.01;   % R
sensor_noise = 0.01;    % Q
errorThreshold = Inf;   % slice out all (actual - expected) > threshold
pSize = 500;            % particle size
particleStateNoise = 0.05;   % noise for spreading particles
particleSensorNoise = 0.1;  % noise for evaluating particles

filterName = "pfdepth";

% For dataStore
sensor_pos = [0.13 0];
load('lab2WallMap2023.mat')
map = lab2WallMap2023;

initialPose = dataStore.truthPose(1,2:end).';
% initialPose = [0; 0; pi/2];

dataStore.ekfMu = initialPose;
dataStore.ekfSigma = [0.05 0 0; 0 0.05 0; 0 0 0.1];
dataStore.GPS = [];

R = process_noise * eye(3);
Q_depth = sensor_noise * eye(9);
Q_gps = sensor_noise * eye(3);

% Particles
sampleUniform = @(minimum, maximum) rand(pSize,1) * (maximum - minimum) + minimum;
initialParticles = zeros(pSize, 3);
initialParticles = initialParticles + initialPose.';
dataStore.particles = initialParticles;
dataStore.weights = 1/pSize + zeros(pSize,1);

% anonymous functions
dynamics = @(x,u) integrateOdom(x, u(1), u(2));
dynamicsJac = @(x,u) GjacDiffDrive(x, u);
sensorGPS = @(x) hGPS(x);
sensorGPSJac = @(x) HjacGPS(x);
n_rs_rays = 9;
angles_degree = linspace(27, -27, n_rs_rays);
angles = angles_degree * pi / 180;
sensorDepth = @(x) depthPredict(x, map, sensor_pos, angles.');
sensorDepthJac = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays);

% Data Processing
dataStore.deadReck = integrateOdom(initialPose, dataStore.odometry(:,2), dataStore.odometry(:,3));

for i = 1:size(dataStore.truthPose,1)
    % Collect some data from dataStore
    tp = dataStore.truthPose(i, 2:end).';
    % dataStore.GPS
    z_gps = tp + normrnd(zeros(size(tp)), sqrt(sensor_noise));
    dataStore.GPS(:,:,i) = z_gps;
    % get control and detph
    u = dataStore.odometry(i, 2:end).';
    z_depth = dataStore.rsdepth(i,3:end).';

    if filterName == "ekfgps"
        [mu_next, sigma_next] = ...
            EKF(dataStore.ekfMu(:,:,end), dataStore.ekfSigma(:,:,end), u, z_gps, ...
                dynamics, dynamicsJac, R, sensorGPS, sensorGPSJac, Q_gps, errorThreshold);
        dataStore.ekfMu(:,:,end+1) = mu_next;
        dataStore.ekfSigma(:,:,end+1) = sigma_next;

    elseif filterName == "ekfdepth"
        [mu_next, sigma_next] = ...
            EKF(dataStore.ekfMu(:,:,end), dataStore.ekfSigma(:,:,end), u, z_depth, ...
                dynamics, dynamicsJac, R, sensorDepth, sensorDepthJac, Q_depth, errorThreshold);
        dataStore.ekfMu(:,:,end+1) = mu_next;
        dataStore.ekfSigma(:,:,end+1) = sigma_next;

    elseif filterName == "pfdepth"
        currentParticles = dataStore.particles(:,:,end);
        currentWeights = dataStore.weights(:,:,end);
        [dataStore.particles(:,:,end+1), dataStore.weights(:,:,end+1)] = ...
            PF(currentParticles, currentWeights, particleStateNoise, particleSensorNoise, u, z_depth, dynamics, sensorDepth);
    else
        disp("Unrocognized filterName, use [ekfgps, ekfdepth, or pfdepth")
    end
end