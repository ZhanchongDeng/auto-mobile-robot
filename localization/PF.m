function [particlesNew, weightsNew] = PF(particles, weights, noiseState, noiseSensor,...
    u, z, dynamics_model, sensor_model)
% PF: Particle filter one step update 
% 
% INPUT:
%   particles           -   N by 3, set of particles, each with its own Pose [x y theta]
%   weights             -   N by 1, weights for each particles
%   u                   -   control input [d; phi]
%   z                   -   measurement (could be GPS pose [x; y; theta] or depth K x 1)
%   dynamics_model      -   g(mu, u) -> next mu
%   sensor_model        -   h(mu) -> expected z
% 
% OUTPUT:
%   particlesNew        -   N by 3, set of particles, each with its own Pose [x y theta]

N = size(particles,1);
d = size(particles,2);

particlesNew = zeros(N,d);
weightsNew = zeros(N,1);

candidates = datasample(particles, N, 1, Replace = true, Weights = weights);
for i = 1:N
    predictedMu = dynamics_model(candidates(i,:)', u);
    particlesNew(i,:) = normrnd(predictedMu, noiseState);
    allSensorWeights = normpdf(z, sensor_model(particlesNew(i,:)'), noiseSensor);
    % remove 0 cause easily all 0 with such small std
    %allSensorWeights = allSensorWeights(allSensorWeights~=0);
    allSensorWeights = allSensorWeights + 1e-5;
    % product of all elements in all Sensor Weights
    weightsNew(i,:) = prod(allSensorWeights, 'all');
end

end