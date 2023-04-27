function [particlesNew, weightsNew] = PF_beacon(particles, weights, noiseState, noiseSensor, ...
        u, z_depth, z_beacon, dynamics_model, sensor_model)
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
    %
    % OUTPUT:
    %   particlesNew        -   N by 3, set of particles, each with its own Pose [x y theta]

    N = size(particles, 1);
    d = size(particles, 2);

    particlesNew = zeros(N, d);
    weightsNew = zeros(N, 1);

    z = [z_beacon; z_depth];

    candidates = datasample(particles, N, 1, Replace = true, Weights = weights);

    for i = 1:N
        predictedMu = dynamics_model(candidates(i, :)', u);
        particlesNew(i, :) = normrnd(predictedMu, noiseState);
        z = z(~isnan(z));
        expected_z = sensor_model(particlesNew(i, :)');
        expected_z = expected_z(~isnan(z));
        allSensorWeights = normpdf(z, expected_z, noiseSensor);
        % remove 0 cause easily all 0 with such small std
        %allSensorWeights = allSensorWeights(allSensorWeights~=0);
        allSensorWeights = allSensorWeights +1e-10;
        % product of all elements in all Sensor Weights
        weightsNew(i, :) = prod(allSensorWeights, 'all');
    end

end
