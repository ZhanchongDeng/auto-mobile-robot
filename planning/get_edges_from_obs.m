function [edges] = get_edges_from_obs(obs)
% Collect all edges from obstacles
    edges = [];
    for i=1:length(obs)
        ob = obs{i};
        for j=1:size(ob,1)-1
            edges(end+1, :) = [ob(j,1), ob(j,2), ob(j+1,1), ob(j+1,2)];
        end
    end
end