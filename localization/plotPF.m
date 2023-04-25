g = [];
map = lab2WallMap2023;
for i = 1:length(map)
    wall = reshape(map(i,:), 2,2);
    g(1) = plot(wall(1,:), wall(2,:), 'black');
    hold on

end
g(2) = plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3), 'g');
g(3) = quiver(dataStore.particles(:,1,1), dataStore.particles(:,2,1), ...
    cos(dataStore.particles(:,3,1)), sin(dataStore.particles(:,3,1)), 0.5);
% ylim([-5 5])
g(4) = scatter(dataStore.particles(:,1,end), dataStore.particles(:,2,end), 'b', 'filled');

bestParticles = zeros(size(dataStore.particles, 3), 5, 3);
averageParticles = zeros(size(dataStore.particles, 3), 3);
for t = 1:size(dataStore.particles,3)
    weights = dataStore.weights(:,:,t);
    particles = dataStore.particles(:,:,t);
    [out, idx] = sort(weights.');
    bestParticles(t, :, :) = particles(idx(1:5),:);
    averageParticles(t,:) = mean(particles, 1);
end


% for i = 5:5
%     g(i) = plot(bestParticles(:,i-2,1), bestParticles(:,i-2,2));
% end

g(5) = plot(averageParticles(:,1), averageParticles(:,2));
title("Particle Filter Top 5 Trajectories (n=50)")
xlabel("x positions")
ylabel("y positions")

%legend(g, 'wall', 'truth pose', 'Final particles')
legend(g, 'wall', 'truth pose', 'initial', 'final','1st traj')