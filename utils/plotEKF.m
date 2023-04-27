g = [];
load('practiceMap_4credits/practiceMap_4credits_2023.mat')
for i = 1:length(map)
    wall = reshape(map(i,:), 2,2);
    g(1) = plot(wall(1,:), wall(2,:), 'black');
    hold on

end
g(2) = plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3), 'g');
g(3) = plot(squeeze(dataStore.ekfMu(1,:,:)), squeeze(dataStore.ekfMu(2,:,:)), 'c.');
% g(3) = plot(squeeze(ekfMu(:,1)), squeeze(ekfMu(:,2)), 'c.');
% g(4) = plot(squeeze(dataStore2.ekfMu(1,:,:)), squeeze(dataStore2.ekfMu(2,:,:)), 'm');

covIdx = linspace(1,size(dataStore.ekfMu, 3)-1, 5);
for i = 1:5
    idx = floor(covIdx(i));
    pose = squeeze(dataStore.ekfMu(:,:,idx));
    sigma = squeeze(dataStore.ekfSigma(:,:,idx));
    g(4) = plotCovEllipse(pose(1:2),sigma(1:2,1:2),1,[{'color'},{'b'}]);

    % pose2 = squeeze(ekfMu(idx,:));
    % sigma2 = squeeze(ekfSigma(idx,:,:));
    % g(4) = plotCovEllipse(pose2(1:2),sigma2(1:2,1:2),1,[{'color'},{'b'}]);
end
% % plot initial cov
% pose = squeeze(dataStore.ekfMu(:,:,1));
% sigma = squeeze(dataStore.ekfSigma(:,:,1));
% g(5) = plotCovEllipse(pose(1:2),sigma(1:2,1:2),1);
% % plot final cov
% pose = squeeze(dataStore.ekfMu(:,:,end));
% sigma = squeeze(dataStore.ekfSigma(:,:,end));
% g(6) = plotCovEllipse(pose(1:2),sigma(1:2,1:2),1);

title("EKF with depth and Beacon")
legend(g, 'Walls', 'Truth Pose', 'EKF', 'Sigma')
% title("EKF Depth compare Code")
% legend(g, 'Walls', 'Truth Pose','EKF Depth zd87', 'EKF Depth ml2226', 'Sigma zd87', 'Sigma ml2226')
% title("EKF GPS")
% legend(g, 'Walls', 'Truth Pose', 'Dead Reckon', 'EKF GPS', 'Sigma')
xlabel("x position")
ylabel("y position")
hold off