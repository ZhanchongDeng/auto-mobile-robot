function [pose] = matchWaypoints(particles, weights, waypoints, k)
    % match closest waypoint to the mean of the top k weighted particles
    %
    % INPUTS:
    %   particles   -   Nx3 matrix of particles
    %   weights     -   Nx1 vector of weights
    %   waypoints   -   Mx3 matrix of waypoints
    %   k           -   top k weighted particles to average
    %
    % OUTPUTS:
    %   pose        -   1x3 vector of the closest waypoint to the top k mean

    [out, idx] = sort(weights.');
    topKParticles = particles(idx(1:k), :);
    pose = mean(topKParticles);

    dist = vecnorm(waypoints - pose(1:2), 2, 2);
    [out, idx] = min(dist);
    pose(1:2) = waypoints(idx(1), :);
end
