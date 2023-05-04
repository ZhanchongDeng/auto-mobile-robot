function [pose] = topKPose(particles, weights, k)
    % match closest waypoint to the mean of the top k weighted particles
    %
    % INPUTS:
    %   particles   -   Nx3 matrix of particles
    %   waypoints   -   Mx3 matrix of waypoints
    %   k           -   top k weighted particles to average
    %
    % OUTPUTS:
    %   pose        -   1x3 vector of the closest waypoint to the top k mean

    [out, idx] = sort(weights.');
    topKParticles = particles(idx(1:k), :);
    pose = mean(topKParticles);

end
