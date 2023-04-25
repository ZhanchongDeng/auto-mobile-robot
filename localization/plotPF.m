function [g] = plotPF(dataStore, map)
    g = [];
    for i = 1:length(map)
        wall = reshape(map(i,:), 2,2);
        g(1) = plot(wall(1,:), wall(2,:), 'black');
        hold on
    
    end
    x = dataStore.particles(:,1,1);
    y = dataStore.particles(:,2,1);
    theta = dataStore.particles(:,3,1);
    g(2) = quiver(x, y, cos(theta), sin(theta), 'Color', 'y');
    g(3) = quiver(dataStore.truthPose(end,2), dataStore.truthPose(end,3), ...
        cos(dataStore.truthPose(end,4)), sin(dataStore.truthPose(end,4)), 'g');
    x = dataStore.particles(:,1,end);
    y = dataStore.particles(:,2,end);
    theta = dataStore.particles(:,3,end);
    g(4) = quiver(x, y, cos(theta), sin(theta), 'Color', 'c');
%     g(5) = quiver(mean(x), mean(y), cos(mean(theta)), sin(mean(theta)), 'Color', 'b');
    g(5) = quiver(dataStore.predictedPose(1), dataStore.predictedPose(2), ...
        cos(dataStore.predictedPose(3)), sin(dataStore.predictedPose(3)), 'Color', 'b');
    
    % bestParticles = zeros(size(dataStore.particles, 3), 5, 3);
    % averageParticles = zeros(size(dataStore.particles, 3), 3);
    % for t = 1:size(dataStore.particles,3)
    %     weights = dataStore.weights(:,:,t);
    %     particles = dataStore.particles(:,:,t);
    %     [out, idx] = sort(weights.');
    %     bestParticles(t, :, :) = particles(idx(1:5),:);
    %     averageParticles(t,:) = mean(particles, 1);
    % end
    
    
    % for i = 5:5
    %     g(i) = plot(bestParticles(:,i-2,1), bestParticles(:,i-2,2));
    % end
    % g(5) = plot(averageParticles(:,1), averageParticles(:,2));
    
    title("Initial Localization")
    xlabel("x positions")
    ylabel("y positions")
    
    legend(g, 'Wall', 'Initial Particles', 'True Pose', 'Final Particles', 'Predicted Pose')
    % legend(g, 'wall', 'truth pose', 'initial', 'final','1st traj')
end