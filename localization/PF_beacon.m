function [particlesNew, weightsNew] = PF_beacon(particles, weights, noiseState, noiseBeacon, noiseDepth, ...
        u, z_depth, z_beacon, dynamics_model, sensor_model, doSample)
    % PF_beacon: Particle filter with beacon one step update
    %
    % INPUT:
    %   particles           -   N by 3, set of particles, each with its own Pose [x y theta]
    %   weights             -   N by 1, weights for each particles
    %   u                   -   control input [d; phi]
    %   z_depth             -   measurement depth K x 1, -1 means optional wall
    %   z_beacon            -   measurement beacon 2N x 1, NaN means no measurement
    %   dynamics_model      -   g(mu, u) -> next mu
    %   sensor_model        -   h(mu) -> expected z
    %   doSample            -   if we want to sample based on weight.
    %
    % OUTPUT:
    %   particlesNew        -   N by 3, set of particles, each with its own Pose [x y theta]

    N = size(particles, 1);
    d = size(particles, 2);
    w = 1e100;

    particlesNew = zeros(N, d);
    weightsNew = zeros(N, 1);

    % measurement is first 12 beacon, then 9 depth
    z = [z_beacon; z_depth];
    % noise sensor is first 12 noiseBeacon, then 9 noiseDepth
    noiseSensor = [(noiseBeacon * ones(size(z_beacon, 1), 1)); (noiseDepth * ones(9, 1))];

    if doSample
        candidates = datasample(particles, N, 1, Replace = true, Weights = weights);
    else
        candidates = particles;
    end

    for i = 1:N
        %% dynamics
        predictedMu = dynamics_model(candidates(i, :)', u);
        particlesNew(i, :) = normrnd(predictedMu, noiseState);

        %% Prepare z and expected_z
        expected_z = sensor_model(particlesNew(i, :)');
        % NaN in Actual Beacon means no measurement, should ignore (slice out)
        ignoreNanBeacon= isnan(z);
        % -1 in Expected Depth means particle see optional wall, should ignore (slice out)
        ignoreOptionalWall = (expected_z == -1);
        expected_z(ignoreOptionalWall) = z(ignoreOptionalWall);
        keep = (~ignoreNanBeacon);
        %% Slicing
        sliced_z = z(keep);
        expected_z = expected_z(keep);
        size(noiseSensor)
        noise = noiseSensor(keep);
        
        % NaN in Expected Beacon means particle don't see a beacon, should punish (large measurement = weight 0)
        punishNanBeacon = isnan(expected_z);
        expected_z(punishNanBeacon) = 20;

        %% Evaluation Using Gaussian
        % normalize expected_z to mean 0 and std 1
        expected_z = (expected_z - sliced_z) ./ noise;
        % pdf should range from [0 - 0.39]
        allSensorWeights = normpdf(0, expected_z, 1);
        allSensorWeights(punishNanBeacon | (allSensorWeights < 1e-100)) = 1e-100;
        % product of all elements in all Sensor Weights
        weightsNew(i,:) = prod(allSensorWeights, 'all') * 1e100;
    end
    % normalize particles
    weightsNew = weightsNew ./ sum(weightsNew);
    % top 5 particles based on weights
    [~, idx] = sort(weightsNew, 'descend');
    % disp('Top 5 particles and weights')
    % particlesNew(idx(1:5), :)
    % weightsNew(idx(1:5), :)
end
