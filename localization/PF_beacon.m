function [particlesNew, weightsNew] = PF_beacon(particles, weights, noiseState, noiseBeacon, noiseDepth, ...
        u, z_depth, z_beacon, dynamics_model, sensor_model, doSample)
    % PF_beacon: Particle filter with beacon one step update
    %
    % INPUT:
    %   particles           -   N by 3, set of particles, each with its own Pose [x y theta]
    %   weights             -   N by 1, weights for each particles
    %   u                   -   control input [d; phi]
    %   z_depth             -   measurement depth K x 1
    %   z_beacon            -   measurement beacon 2N x 1
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

    z = [z_beacon; z_depth];
    nonNanActual = ~isnan(z);
    z = z(nonNanActual);
    % noise sensor is first 12 noiseBeacon, then 9 noiseDepth
    
    noiseSensor = [(noiseBeacon * ones(12, 1)); (noiseDepth * ones(9, 1))];
    noise = noiseSensor(nonNanActual);

    if doSample
        candidates = datasample(particles, N, 1, Replace = true, Weights = weights);
    else
        candidates = particles;
    end

    for i = 1:N
        % dynamics
        predictedMu = dynamics_model(candidates(i, :)', u);
        particlesNew(i, :) = normrnd(predictedMu, noiseState);
        % get expected_z, slice out beacons not seeing
        expected_z = sensor_model(particlesNew(i, :)');
        expected_z = expected_z(nonNanActual);
        % record NaNs from expected itself
        isNanExpected = isnan(expected_z);
        % avoid NaN math, will slice out these results anyways
        expected_z(isNanExpected) = 1e10;
        % normalize expected_z
        expected_z = (expected_z - z) ./ noise;
        % should range from [0 - 0.39]
        allSensorWeights = normpdf(0, expected_z, 1);
        % for nans in expected_z but not in z, punish hard by setting small
        % values
%         allSensorWeights(isNanExpected) = normpdf(0,4,1);
        % product of all elements in all Sensor Weights
        disp('curr_weight')
        weightsNew(i,:) = prod(allSensorWeights, 'all') * 1e100
        
%         weightsNew(i, :) = max(sum(log(allSensorWeights), 'all'), normpdf(0, 4, 1));

    end
    % normalize particles
    disp('normalized weights')
    weightsNew = weightsNew ./ sum(weightsNew)
    % top 5 particles based on weights
    [~, idx] = sort(weightsNew, 'descend');
    disp('Top 5 particles and weights')
    particlesNew(idx(1:5), :)
    weightsNew(idx(1:5), :)
end
