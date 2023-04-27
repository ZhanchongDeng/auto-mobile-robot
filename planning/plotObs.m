function [pmap] = plotObs(obs, mapLimits, figHdl)
% PLOTOBS: Draw a map including obstacles and map boundary
    if ~exist('figHdl','var') || isempty(figHdl), figHdl = gcf; end
    color = [0 1 0];
    edges = get_edges_from_obs(obs);
    pmap = plot([edges(:,1) edges(:,3)]', [edges(:,2) edges(:,4)]', 'Color', color, 'LineWidth', 3);
    hold on
    
     % Define map limits
    vxMin = mapLimits(1); 
    vyMin = mapLimits(2);
    vxMax = mapLimits(3);
    vyMax = mapLimits(4);

    % Draw the boundary
    line([vxMin vxMax],[vyMin vyMin],'LineWidth',3,'Color',color);
    line([vxMax vxMax],[vyMin vyMax],'LineWidth',3,'Color',color)
    line([vxMax vxMin],[vyMax vyMax],'LineWidth',3,'Color',color)
    line([vxMin vxMin],[vyMax vyMin],'LineWidth',3,'Color',color)

end
