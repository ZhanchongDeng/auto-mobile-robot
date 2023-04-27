function [pmap] = plotmapopts(map, varargin)
%     for i=1:size(map, 1)
%         plot([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'color', 'Green', 'HandleVisibility','off', 'LineWidth', '5');
%         hold on
%     end
%     pmap = plot([map(:,1) map(:,3)]', [map(:,2) map(:,4)]', 'b--', 'LineWidth', 1);
%     pmap = plot([map(:,1) map(:,3)]', [map(:,2) map(:,4)]', 'g', 'LineWidth', 1);
    pmap = plot([map(:,1) map(:,3)]', [map(:,2) map(:,4)]', varargin{:});
    
end