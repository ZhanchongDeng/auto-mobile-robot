function [initialParticles] = particlesFromWaypoints(pSize, waypoints)
% particlesFromWaypoints: Generates equal amount of particles for each
% waypoint, pointing toward all direction.
%
%   [particles] = particlesFromWaypoints(pSize, waypoints, radius)
%
%   INPUTS
%       pSize: (1,) Number of particles to generate
%       waypoints: (N, 2) Waypoints to generate particles around
%       radius: (1,) Radius of the sphere around the waypoint
%
%   OUTPUTS
%       initialParticles: (N,3) Generated particles
%
    sampleUniform = @(minimum, maximum, size) rand(size,1) * (maximum - minimum) + minimum;
    initialParticles = zeros(pSize, 3);

    % generate indexes for each waypoint
    indexes = floor(linspace(1, pSize, size(waypoints, 1) + 1));

    for i = 1:size(waypoints, 1)
        % generate particles
        cur_size = indexes(i+1) - indexes(i) + 1;
        particles = zeros(cur_size, 3);
        particles(:,1) = waypoints(i,1);
        particles(:,2) = waypoints(i,2);
        particles(:,3) = sampleUniform(-pi, pi , cur_size);
        initialParticles(indexes(i):indexes(i+1),:) = particles;
    end

end